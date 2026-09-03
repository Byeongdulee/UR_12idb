"""decodeQR() must keep the shape pyzbar returned, now backed by zxing-cpp.

Callers compare payloads against byte literals (b'stv1', b'sav') and treat
.polygon points as adjacent corners, so those contracts are pinned here rather
than just "does it decode".
"""
import io as _io

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
segno = pytest.importorskip("segno")          # pure-Python QR generator
pytest.importorskip("zxingcpp")
Image = pytest.importorskip("PIL.Image")

from common.urcamera import decodeQR  # noqa: E402

PAYLOAD = "sav-001"


def qr_image(text=PAYLOAD, scale=8, border=6, micro=False):
    """Render `text` as a QR code in a BGR array, the way a camera frame looks."""
    buf = _io.BytesIO()
    make = segno.make if micro else segno.make_qr
    make(text, error="m").save(buf, kind="png", scale=scale, border=border)
    grey = np.array(Image.open(buf).convert("L"))
    return cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)


def rotated(img, degrees):
    h, w = img.shape[:2]
    matrix = cv2.getRotationMatrix2D((w // 2, h // 2), degrees, 1.0)
    return cv2.warpAffine(img, matrix, (w, h), borderValue=(255, 255, 255))


def test_returns_empty_list_when_no_code_present():
    assert decodeQR(np.full((128, 128, 3), 255, np.uint8)) == []


def test_payload_is_bytes_not_str():
    # camera_tools compares `data == b'sav'` and urcamera does
    # centindx.index(b'stv1'); a str payload would silently never match.
    found = decodeQR(qr_image())
    assert len(found) == 1
    assert found[0].data == PAYLOAD.encode()
    assert isinstance(found[0].data, bytes)


def test_rect_is_the_bounding_box():
    code = decodeQR(qr_image())[0]
    left, top, width, height = code.rect          # showQRcode unpacks it this way
    assert code.rect.left == left and code.rect.height == height
    assert width > 0 and height > 0
    xs = [point.x for point in code.polygon]
    ys = [point.y for point in code.polygon]
    assert (left, top) == (min(xs), min(ys))
    assert (width, height) == (max(xs) - min(xs), max(ys) - min(ys))


def test_polygon_has_four_corners_in_cyclic_order():
    code = decodeQR(qr_image())[0]
    assert len(code.polygon) == 4
    # Consecutive points must be adjacent corners, so all four edges of a
    # square code are about equal -- decodeQR averages them for distance.
    edges = [
        np.hypot(code.polygon[i].x - code.polygon[(i + 1) % 4].x,
                 code.polygon[i].y - code.polygon[(i + 1) % 4].y)
        for i in range(4)
    ]
    assert max(edges) / min(edges) < 1.1, "polygon is not in perimeter order: %s" % edges


def test_decodes_a_rotated_code():
    # The wrist camera never sees a holder perfectly square-on.
    found = decodeQR(rotated(qr_image(), 33))
    assert [c.data for c in found] == [PAYLOAD.encode()]


def test_decodes_an_inverted_code():
    # Light-on-dark; pyzbar and OpenCV both miss this.
    found = decodeQR(255 - qr_image())
    assert [c.data for c in found] == [PAYLOAD.encode()]


def test_decodes_micro_qr():
    # Micro QR fits the same payload in a smaller mark; OpenCV cannot read it.
    found = decodeQR(qr_image("stv1", micro=True))
    assert [c.data for c in found] == [b"stv1"]
    assert "Micro" in found[0].type


def test_finds_both_codes_in_a_two_qr_frame():
    # urcamera's "2QR" path expects one entry per code, and looks them up by
    # payload with centindx.index(b'stv0') / b'stv1'.
    left, right = qr_image("stv0"), qr_image("stv1")
    frame = np.full((left.shape[0], left.shape[1] * 2 + 60, 3), 255, np.uint8)
    frame[:, :left.shape[1]] = left
    frame[:, left.shape[1] + 60:] = right

    payloads = sorted(code.data for code in decodeQR(frame))

    assert payloads == [b"stv0", b"stv1"]
