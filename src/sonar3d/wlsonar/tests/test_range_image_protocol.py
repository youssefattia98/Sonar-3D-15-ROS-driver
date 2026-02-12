import io
import struct
from typing import Dict, List

import pytest
from google.protobuf.timestamp_pb2 import Timestamp

from wlsonar import range_image_protocol
from wlsonar.range_image_protocol import (
    BitmapImageGreyscale8,
    Header,
    RangeImage,
)

TEST_RANGE_IMAGE = RangeImage(
    header=Header(
        timestamp=Timestamp(seconds=0, nanos=42),
        sequence_id=111,
    ),
    width=256,
    height=64,
    fov_horizontal=40.0,
    fov_vertical=40.0,
    image_pixel_scale=50.0 / 30000.0,
    image_pixel_data=[0] * (256 * 64),
)


def assert_range_images_equal(
    a: RangeImage,
    b: RangeImage,
) -> None:
    try:
        assert a.header.timestamp.seconds == b.header.timestamp.seconds
        assert a.header.timestamp.nanos == b.header.timestamp.nanos
        assert a.header.sequence_id == b.header.sequence_id
        assert a.width == b.width
        assert a.height == b.height
        assert a.fov_horizontal == b.fov_horizontal
        assert a.fov_vertical == b.fov_vertical
        assert a.image_pixel_scale == b.image_pixel_scale
        assert a.image_pixel_data == b.image_pixel_data
    except AssertionError as e:
        raise AssertionError("RangeImages are not equal") from e


def test_range_image_protocol__bad_id(
    request: pytest.FixtureRequest,
) -> None:
    # bytes below are for a PNG file followed by zeros... should fail as not a range image packet
    bad = bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0])
    with pytest.raises(range_image_protocol.BadIDError):
        _ = range_image_protocol.unpackb(bad)


def test_range_image_protocol__small(
    request: pytest.FixtureRequest,
) -> None:
    # some tests for small inputs to smoke test that we don't return successfully on invalid data

    tests = [
        bytes([]),
        bytes([0x1, 0x2]),
        range_image_protocol.ProtocolVersion.RIP1.bytes(),
        bytes([*range_image_protocol.ProtocolVersion.RIP1.bytes(), 0]),
        bytes([*range_image_protocol.ProtocolVersion.RIP1.bytes(), 0, 0, 0, 0]),
        bytes([*range_image_protocol.ProtocolVersion.RIP1.bytes(), 0, 0, 0, 0, 0]),
        bytes([*range_image_protocol.ProtocolVersion.RIP1.bytes(), 0, 0, 0, 0, 0, 0, 0, 0]),
    ]

    for test_case in tests:
        with pytest.raises(Exception):
            _ = range_image_protocol.unpackb(test_case)


def test_range_image_protocol__packb_unpackb__happy_path(request: pytest.FixtureRequest) -> None:
    decoded = range_image_protocol.unpackb(range_image_protocol.packb(TEST_RANGE_IMAGE))

    assert isinstance(decoded, RangeImage)

    assert_range_images_equal(TEST_RANGE_IMAGE, decoded)


def test_range_image_protocol__bad_crc(
    request: pytest.FixtureRequest,
) -> None:
    # assumes dumps & loads of TEST_RANGE_IMAGE is tested to work correctly

    encoded = range_image_protocol.packb(TEST_RANGE_IMAGE)

    # Corrupt the data by flipping a bit inside byte 23
    encoded = bytearray(encoded)
    encoded[23] = encoded[23] & (1 << 4)

    with pytest.raises(range_image_protocol.CRCMismatchError):
        _ = range_image_protocol.unpackb(encoded)


def test_range_image_protocol__packet_length__too_high(
    request: pytest.FixtureRequest,
) -> None:
    # Create a valid packet
    valid_packet = range_image_protocol.packb(TEST_RANGE_IMAGE)

    # corrupt the packet by increasing the packet length by 1
    curr_len = struct.unpack("<I", valid_packet[4:8])[0]
    invalid_packet = valid_packet[:4] + struct.pack("<I", curr_len + 1) + valid_packet[8:]

    # will fail because packet is too small..
    with pytest.raises(Exception):
        _ = range_image_protocol.unpackb(invalid_packet)


def test_range_image_protocol__packet_length__too_low(
    request: pytest.FixtureRequest,
) -> None:
    # Create a valid packet
    valid_packet = range_image_protocol.packb(TEST_RANGE_IMAGE)

    # corrupt the packet by decreasing the packet length by 1
    curr_len = struct.unpack("<I", valid_packet[4:8])[0]
    invalid_packet = valid_packet[:4] + struct.pack("<I", curr_len - 1) + valid_packet[8:]

    # will fail because reading too few bytes from payload, and then reading wrong bytes for crc
    with pytest.raises(Exception):
        _ = range_image_protocol.unpackb(invalid_packet)


def test_range_image_protocol__unpackb_empty_should_raise(
    request: pytest.FixtureRequest,
) -> None:
    with pytest.raises(ValueError):
        _ = range_image_protocol.unpackb(b"")


def test_range_image_protocol__unpack_empty_should_raise_eof(
    request: pytest.FixtureRequest,
) -> None:
    with pytest.raises(EOFError):
        _ = range_image_protocol.unpack(io.BytesIO(b""))


def test_range_image_protocol__unpackb_extra_data_should_raise(
    request: pytest.FixtureRequest,
) -> None:
    # Create a valid packet
    valid_packet = range_image_protocol.packb(TEST_RANGE_IMAGE)

    # add extra data at the end
    invalid_packet = valid_packet + b"\x00\x01\x02\x03"

    with pytest.raises(range_image_protocol.ExtraDataError) as exc_info:
        _ = range_image_protocol.unpackb(invalid_packet)

    assert exc_info.value.extra_data_length == 4


def test_range_image_protocol__unpack_extra_data(
    request: pytest.FixtureRequest,
) -> None:
    # Create a valid packet
    valid_packet = range_image_protocol.packb(TEST_RANGE_IMAGE)

    # add extra data at the end
    invalid_packet = io.BytesIO(valid_packet + b"\x00\x01\x02\x03")

    # first unpack should be fine
    _ = range_image_protocol.unpack(invalid_packet)

    # but second unpack should raise because what is left is not a valid packet
    with pytest.raises(Exception) as e:
        _ = range_image_protocol.unpack(invalid_packet)
    # not EOFError, since that indicates a clean end of file
    assert not isinstance(e, EOFError)


def test_range_image_protocol__unpack_example_data(
    request: pytest.FixtureRequest,
) -> None:
    # simple sanity checks against a real data file

    # maps of seqno to messages
    range_images: Dict[int, List[RangeImage]] = {}
    greyscale_images: Dict[int, List[BitmapImageGreyscale8]] = {}

    # read out all messages
    with open("tests/data/ship_short.sonar", "rb") as f:
        while True:
            try:
                msg = range_image_protocol.unpack(f)
            except EOFError:
                break
            if isinstance(msg, RangeImage):
                if msg.header.sequence_id not in range_images:
                    range_images[msg.header.sequence_id] = []
                range_images[msg.header.sequence_id].append(msg)
            elif isinstance(msg, BitmapImageGreyscale8):
                if msg.header.sequence_id not in greyscale_images:
                    greyscale_images[msg.header.sequence_id] = []
                greyscale_images[msg.header.sequence_id].append(msg)
            else:
                # ignore other message types for this test
                continue

    # assert some known facts about this file

    assert len(range_images) == 6
    assert len(greyscale_images) == 6

    known_sequence_numbers = [4448, 4449, 4450, 4451, 4452, 4453]

    for seq_no in known_sequence_numbers:
        assert seq_no in range_images
        assert seq_no in greyscale_images
