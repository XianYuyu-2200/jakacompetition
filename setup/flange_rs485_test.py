#!/usr/bin/env python3
"""法兰 RS485 交互收发。

连接控制器 10001 后一直保持，不移动机械臂。
你输入一帧就立刻从通道 0（RS485H，黄 A+ / 粉 B-）原样发出，
控制器从这条连接上返回的内容会马上打印。不会编造串口回包。

  python3 setup/flange_rs485_test.py

输入:
  status / open / close
  7B 01 00 00 00 00 00 00 00 7A 7D
  7B01000000000000007A7D
  quit
"""

import json
import select
import socket
import sys

IP = "192.168.0.102"
PORT = 10001
CHANNEL = 0  # 0 = RS485H 黄/粉, 1 = RS485L 棕/白

# WHEELTEC 夹爪，11 字节，BCC = 前 9 字节异或，尾 7D。
FRAMES = {
    "status": bytes.fromhex("7B 01 00 00 00 00 00 00 00 7A 7D"),
    "close": bytes.fromhex("7B 01 02 01 20 46 50 01 2C 62 7D"),
    "open": bytes.fromhex("7B 01 02 00 20 46 50 01 2C 63 7D"),
}


def hex_of(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def parse_line(line: str):
    text = line.strip()
    if not text:
        return None
    low = text.lower()
    if low in ("quit", "exit", "q"):
        return "quit", b""
    if low in FRAMES:
        return low, FRAMES[low]
    compact = text.replace(",", " ").split()
    if len(compact) == 1 and len(compact[0]) >= 2 and len(compact[0]) % 2 == 0:
        try:
            return "hex", bytes.fromhex(compact[0])
        except ValueError:
            pass
    try:
        return "hex", bytes.fromhex(text.replace(",", " "))
    except ValueError as exc:
        raise ValueError(f"无法解析: {text}") from exc


def byte_lists(value, key=""):
    found = []
    if isinstance(value, dict):
        for item_key, item in value.items():
            if item_key in ("cmdBuf",):
                continue
            found.extend(byte_lists(item, item_key))
    elif isinstance(value, list) and value and all(isinstance(item, int) and 0 <= item <= 255 for item in value):
        found.append((key or "list", bytes(value)))
    elif isinstance(value, list):
        for item in value:
            found.extend(byte_lists(item, key))
    return found


class FlangeRS485:
    def __init__(self):
        self.sock = None
        self.buf = bytearray()

    def connect(self):
        self.sock = socket.create_connection((IP, PORT), timeout=3)
        self.sock.setblocking(False)
        print(f"已连接 {IP}:{PORT}，通道 {CHANNEL} = RS485H")
        self.request({"cmdName": "get_version"})
        self.request({
            "cmdName": "set_rs485_chn_comm",
            "chn_id": CHANNEL,
            "slaveId": 1,
            "baudrate": 115200,
            "databit": 8,
            "stopbit": 1,
            "parity": 78,
        })
        self.request({"cmdName": "set_rs485_chn_mode", "chnId": CHANNEL, "chnMode": 1})
        self.request({"cmdName": "get_rs485_chn_mode", "chnId": CHANNEL})

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def request(self, obj):
        raw = json.dumps(obj, separators=(",", ":")).encode()
        self.sock.sendall(raw)
        messages = self.recv_for(0.8)
        if not messages:
            print("RX 超时，控制器没有返回")
        for message in messages:
            self.show(message)
        return messages

    def send_frame(self, name, data: bytes):
        print(f"TX {name} [{len(data)}] {hex_of(data)}")
        self.sock.sendall(json.dumps({
            "cmdName": "send_tio_rs_command",
            "chn_id": CHANNEL,
            "cmdBuf": list(data),
        }, separators=(",", ":")).encode())

    def recv_for(self, seconds):
        messages = []
        while True:
            ready, _, _ = select.select([self.sock], [], [], seconds)
            if not ready:
                break
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("控制器断开了 10001")
            self.buf.extend(chunk)
            messages.extend(self._pull())
            seconds = 0.05
        return messages

    def drain(self):
        return self.recv_for(0)

    def _pull(self):
        messages = []
        text = self.buf.decode("utf-8", "replace")
        decoder = json.JSONDecoder()
        index = 0
        while True:
            while index < len(text) and text[index].isspace():
                index += 1
            if index >= len(text):
                self.buf.clear()
                break
            try:
                obj, end = decoder.raw_decode(text, index)
            except json.JSONDecodeError:
                self.buf = bytearray(text[index:].encode())
                break
            messages.append(obj)
            index = end
        else:
            self.buf.clear()
        if index >= len(text):
            self.buf.clear()
        return messages

    def show(self, message):
        print("RX " + json.dumps(message, ensure_ascii=False, separators=(",", ":")))
        payloads = byte_lists(message)
        if payloads:
            for key, data in payloads:
                print(f"串口字节 {key} [{len(data)}] {hex_of(data)}")
        elif message.get("cmdName") == "send_tio_rs_command":
            code = str(message.get("errorCode"))
            if code == "0":
                print("串口字节: 无。控制器确认已发出，应答里没有夹爪回包")
            else:
                print(f"串口字节: 无。发送失败 errorCode={code} {message.get('errorMsg', '')}")


def help_text():
    print("输入 status / open / close，或任意十六进制。quit 退出。")
    print("程序会一直挂着；控制器随后推上来的数据也会立刻打印。")


def main():
    link = FlangeRS485()
    link.connect()
    help_text()
    pending = " ".join(sys.argv[1:]).strip()
    try:
        while True:
            if pending:
                line, pending = pending, ""
            else:
                sys.stdout.write("> ")
                sys.stdout.flush()
                try:
                    ready, _, _ = select.select([sys.stdin, link.sock], [], [], None)
                except KeyboardInterrupt:
                    print()
                    break
                if link.sock in ready:
                    try:
                        incoming = link.drain()
                    except ConnectionError as exc:
                        print()
                        print(exc)
                        break
                    if incoming:
                        print()
                        for message in incoming:
                            link.show(message)
                if sys.stdin not in ready:
                    continue
                line = sys.stdin.readline()
                if line == "":
                    print()
                    break
            try:
                parsed = parse_line(line)
            except ValueError as exc:
                print(exc)
                continue
            if parsed is None:
                continue
            name, data = parsed
            if name == "quit":
                break
            try:
                link.send_frame(name, data)
                for message in link.recv_for(0.8):
                    link.show(message)
            except (ConnectionError, OSError) as exc:
                print(f"发送失败: {exc}")
                break
    finally:
        link.close()
        print("已断开")


if __name__ == "__main__":
    main()
