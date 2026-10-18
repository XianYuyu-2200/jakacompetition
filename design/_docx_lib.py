"""按需加载 python-docx。

本机没有 pip,所以找不到 python-docx 时会从 PyPI 下载官方 wheel 解压到
design/.vendor/(已加入 .gitignore),再导入。纯 Python 依赖 lxml,本机已有。
"""
import json
import os
import shutil
import subprocess
import sys
import zipfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_VENDOR = os.path.join(_HERE, ".vendor")


def _download(url, dest):
    """优先用 curl(本机 urllib 的 TLS 握手会超时), 否则回退 urllib。"""
    curl = shutil.which("curl")
    if curl:
        subprocess.run([curl, "-fsSL", "--max-time", "120", "-o", dest, url],
                       check=True)
        return
    import urllib.request
    urllib.request.urlretrieve(url, dest)


def _try_import():
    try:
        import docx  # noqa: F401
        return True
    except ImportError:
        return False


def ensure():
    if _VENDOR not in sys.path:
        sys.path.insert(0, _VENDOR)
    if _try_import():
        return
    os.makedirs(_VENDOR, exist_ok=True)
    print("[docx] python-docx 不可用, 正在从 PyPI 下载 ...")
    meta_path = os.path.join(_VENDOR, "_index.json")
    _download("https://pypi.org/pypi/python-docx/json", meta_path)
    with open(meta_path, encoding="utf-8") as fh:
        meta = json.load(fh)
    url = next(f["url"] for f in meta["urls"] if f["filename"].endswith(".whl"))
    whl = os.path.join(_VENDOR, "_python_docx.whl")
    _download(url, whl)
    with zipfile.ZipFile(whl) as z:
        z.extractall(_VENDOR)
    os.unlink(whl)
    if not _try_import():
        raise RuntimeError("python-docx 安装失败, 请手动安装后再试")
    print("[docx] 已就绪:", url.rsplit("/", 1)[-1])


def load():
    ensure()
    import docx
    return docx
