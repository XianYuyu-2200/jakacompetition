"""Unreadable and ambiguous images must not select a station or start motion."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from jaka_competition_kit import qr, reference
from jaka_competition_kit.ref_track1 import Track1Reference


@pytest.mark.parametrize('image', [None, np.zeros((0, 0), dtype=np.uint8),
                                    np.zeros((120, 120), dtype=np.uint8)])
def test_empty_or_blank_image(image):
    assert qr.decode_image(image) == ''


def test_missing_image(tmp_path):
    assert qr.decode_image(str(tmp_path / 'missing.png')) == ''
    path = tmp_path / 'corrupt.png'
    path.write_bytes(b'not an image')
    assert qr.decode_image(str(path)) == ''


@pytest.mark.parametrize('payloads,expected', [
    ([b'432165', b'123456'], ''), ([b'432165', b'432165'], '432165'),
    ([b'\xff'], ''), ([b'432165', b'\xff'], ''), ([], ''),
])
def test_decoder_rejects_conflicting_or_non_utf8_payloads(monkeypatch, payloads, expected):
    import pyzbar.pyzbar as zbar
    decode = Mock(return_value=[SimpleNamespace(data=p) for p in payloads])
    monkeypatch.setattr(zbar, 'decode', decode)
    assert qr.decode_image(np.zeros((10, 10), dtype=np.uint8)) == expected
    assert decode.call_args.kwargs['symbols'] == [zbar.ZBarSymbol.QRCODE]


def test_real_multi_qr_frame_rejected(tmp_path):
    import cv2
    images = []
    for text in ('432165', '123456'):
        path = tmp_path / f'{text}.png'
        qr.render_png(text, str(path))
        images.append(cv2.imread(str(path), cv2.IMREAD_GRAYSCALE))
    assert qr.decode_image(np.concatenate(images, axis=1)) == ''


@pytest.mark.parametrize('payload', ['', '112345', '12345', '1234567', '023456',
                                     'JAKA1-432165x', 'not-an-order'])
def test_invalid_image_payload_never_calls_motion(tmp_path, payload):
    path = str(tmp_path / 'code.png')
    qr.render_png(payload, path)
    arm = Mock()
    backend = Mock()
    node = SimpleNamespace(
        scene={'track': 1, 'qr_png': path, 'qr_order': [1, 2, 3, 4, 5, 6],
               'objects': [{'station': n} for n in range(1, 7)]},
        get_logger=lambda: Mock(), wait_for_scene=lambda: True,
        wait_for_state=lambda _: True,
        get_parameter=lambda _: SimpleNamespace(value=False), arm=arm, backend=backend)
    node.plan_order = lambda: Track1Reference.plan_order(node)
    reference.ReferenceBase.run(node)
    assert not node.succeeded
    assert arm.mock_calls == [] and backend.mock_calls == []


@pytest.mark.parametrize('payload', [None, 432165, [], {}])
def test_non_text_parse_rejected(payload):
    with pytest.raises(ValueError):
        qr.parse(payload)
