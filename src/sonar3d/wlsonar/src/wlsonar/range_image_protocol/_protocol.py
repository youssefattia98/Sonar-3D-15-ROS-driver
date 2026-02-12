"""Range Image Protocol implementation."""

import io
import struct
import zlib
from enum import Enum
from typing import BinaryIO, Optional, Sequence, Type

import snappy
from google.protobuf.any_pb2 import Any
from google.protobuf.message import Message

from . import (
    BitmapImageGreyscale8,
    Packet,
    RangeImage,
)


class UnknownProtobufTypeError(ValueError):
    """Unknown protobuf type.

    Packets in the Range Image protocol may contain undocumented internal
    messages that you should ignore. UnknownProtobufTypeError is raised by
    parse_rip_packet when such a message is encountered.
    """

    def __init__(self, type_name: str) -> None:
        error_message = (
            "Unknown message type in Range Image Protocol Packet "
            + f"with packet name '{type_name!s}'"
        )
        super().__init__(error_message)
        self.type_name = type_name


class BadIDError(ValueError):
    """BadIDError is raised when the packet identifier is invalid."""

    def __init__(self, magic: bytes) -> None:
        error_message = f"Invalid RIP packet identifier: got {magic!r}"
        super().__init__(error_message)
        self.magic = magic


class CRCMismatchError(ValueError):
    """CRCMismatchError is raised when the packet CRC does not match."""

    def __init__(self, *, expected_crc: int, received_crc: int) -> None:
        error_message = f"CRC mismatch: expected 0x{expected_crc:08x}, got 0x{received_crc:08x}."
        super().__init__(error_message)
        self.expected_crc = expected_crc
        self.received_crc = received_crc


class ExtraDataError(ValueError):
    """ExtraDataError is when there is unexpected extra data left after parsing a packet."""

    def __init__(self, extra_data_length: int) -> None:
        error_message = f"Extra data after parsing packet: {extra_data_length} bytes left."
        super().__init__(error_message)
        self.extra_data_length = extra_data_length


class ProtocolVersion(Enum):
    """Range Image Protocol version.

    Some Sonar 3D-15 releases support sending RIP1 packets. RIP1 will be removed in a future
    release.
    """

    RIP1 = 1
    RIP2 = 2

    def bytes(self) -> bytes:
        """Return the 4-byte identifier for this protocol version."""
        if self == ProtocolVersion.RIP1:
            return b"RIP1"
        elif self == ProtocolVersion.RIP2:
            return b"RIP2"
        else:
            raise ValueError(f"Unknown RIP protocol: {self}")


# all valid protocol identifiers
_all_ids = [protocol.bytes() for protocol in ProtocolVersion]


CURRENT_PROTOCOL = ProtocolVersion.RIP2

# packet length constants
IDENTIFIER_LENGTH = 4
PACKET_LENGTH_LENGTH = 4
CHECKSUM_LENGTH = 4


def pack(
    msg: Message,
    f: BinaryIO,
    *,
    protocol_version: ProtocolVersion = CURRENT_PROTOCOL,
) -> None:
    """Pack msg to file-like object f as a Range Image Protocol packet.

    Args:
        msg: The protobuf message to pack into a Range Image Protocol packet.
        f: A file-like object opened in binary write mode.
        protocol_version: The Range Image Protocol version to use.

    Raises:
        ValueError: If an unknown protocol is specified.
        Exception: May raise other exceptions.
    """
    # Wrap message in Any
    msg_any = Any()
    msg_any.Pack(msg)
    # Create Packet protobuf (replace with your Packet class)
    packet = Packet()
    packet.msg.CopyFrom(msg_any)
    proto_payload = packet.SerializeToString()

    if protocol_version == ProtocolVersion.RIP1:
        rip_payload = proto_payload
    elif protocol_version == ProtocolVersion.RIP2:
        try:
            rip_payload = snappy.compress(proto_payload)
        except Exception as e:
            raise ValueError("Compression failed") from e
    else:
        raise ValueError(f"Unknown RIP protocol: {protocol_version}")

    # Prepare CRC. (CRC calculated by writing simultaneously to buffer and CRC)
    crc = zlib.crc32(b"")

    # Write 4-byte identifier
    magic_id = protocol_version.bytes()
    f.write(magic_id)
    crc = zlib.crc32(magic_id, crc)

    # Write 4-byte packet length
    packet_length = IDENTIFIER_LENGTH + PACKET_LENGTH_LENGTH + len(rip_payload) + CHECKSUM_LENGTH
    length_bytes = struct.pack("<I", packet_length)
    f.write(length_bytes)
    crc = zlib.crc32(length_bytes, crc)

    # Write payload
    f.write(rip_payload)
    crc = zlib.crc32(rip_payload, crc)

    # Write 4-byte CRC
    f.write(struct.pack("<I", crc & 0xFFFFFFFF))


def packb(msg: Message, *, protocol: ProtocolVersion = CURRENT_PROTOCOL) -> bytes:
    """Pack msg to bytes as a Range Image Protocol packet.

    Args:
        msg: The protobuf message to pack into a Range Image Protocol packet.
        protocol: The Range Image Protocol version to use.

    Returns:
        Bytes containing the packed Range Image Protocol packet.
    """
    b = io.BytesIO()
    pack(msg, b, protocol_version=protocol)
    return b.getvalue()


def unpack(
    f: BinaryIO,
    *,
    known_message_types: Optional[Sequence[Type[Message]]] = None,
    max_packet_size: int = 65507,
) -> Message:
    """Unpack the next Range Image Protocol packet from file-like object f.

    On success, f is left at the position immediately after the returned packet.

    Args:
        f: A file-like object opened in binary read mode, positioned at the start of a Range Image
            Protocol packet.
        known_message_types: List of protobuf message classes that are known and can be unpacked
            from the Packet.
        max_packet_size: Maximum allowed packet size in bytes. Max theoretical packet size is 4GB,
            given the 4-byte length field. Default max size is 65507, which is the max size of Range
            Image Protocol packet over UDP.

    Returns:
        The decoded protobuf message. One of the known_message_types.

    Raises:
        UnknownProtobufTypeError: If the message type in the Packet is not one of
            known_message_types.
        BadIDError: If the packet identifier is invalid.
        CRCMismatchError: If the packet CRC does not match.
        EOFError: End of file.
        Exception: May raise other exceptions.
    """
    if known_message_types is None:
        # Default known types
        known_message_types = (RangeImage, BitmapImageGreyscale8)

    # read the first 8 bytes: identifier + total length
    magic_and_packet_length = f.read(IDENTIFIER_LENGTH + PACKET_LENGTH_LENGTH)
    if not magic_and_packet_length:
        # first read was empty: EOF
        raise EOFError()
    if len(magic_and_packet_length) < IDENTIFIER_LENGTH + PACKET_LENGTH_LENGTH:
        raise ValueError(
            "Incomplete Range Image Protocol packet header: expected "
            f"{IDENTIFIER_LENGTH + PACKET_LENGTH_LENGTH} bytes, got {len(magic_and_packet_length)}."
        )

    # First 4 bytes are the identifier
    magic = magic_and_packet_length[:4]
    if magic not in _all_ids:
        raise BadIDError(magic)

    # Next 4 bytes (little-endian) specify the total packet length
    total_length = struct.unpack("<I", magic_and_packet_length[4:8])[0]

    if total_length < IDENTIFIER_LENGTH + PACKET_LENGTH_LENGTH + CHECKSUM_LENGTH:
        raise ValueError(
            f"Range Image Protocol packet length {total_length} is too small to be valid."
        )

    if total_length > max_packet_size:
        raise ValueError(
            f"Range Image Protocol packet length {total_length} exceeds given maximum allowed size "
            f"of {max_packet_size}."
        )

    # Now that we know the total length, read the rest of the packet
    remaining_length = total_length - (IDENTIFIER_LENGTH + PACKET_LENGTH_LENGTH)

    payload_and_crc = f.read(remaining_length)
    if len(payload_and_crc) < remaining_length:
        raise ValueError(
            f"Incomplete Range Image Protocol packet: expected {remaining_length} bytes, "
            f"got {len(payload_and_crc)}."
        )

    payload = payload_and_crc[: remaining_length - CHECKSUM_LENGTH]
    crc_bytes = payload_and_crc[remaining_length - CHECKSUM_LENGTH :]

    # Last 4 bytes in the packet is the CRC32
    crc_received = struct.unpack("<I", crc_bytes)[0]
    crc_calculated = zlib.crc32(magic_and_packet_length + payload) & 0xFFFFFFFF
    if crc_calculated != crc_received:
        raise CRCMismatchError(expected_crc=crc_calculated, received_crc=crc_received)

    # Decompress if RIP2
    if magic == ProtocolVersion.RIP2.bytes():
        try:
            payload = snappy.uncompress(payload)
        except Exception as e:
            raise ValueError("Decompression failed") from e

    if not isinstance(payload, bytes):
        raise ValueError("Decompression did not return bytes.")

    # Create a top-level Packet object
    packet = Packet()
    try:
        packet.ParseFromString(payload)
    except Exception as e:
        raise ValueError("Protobuf parse error") from e

    if not packet.IsInitialized():
        raise ValueError("Uninitialized packet.")

    if not packet.msg.IsInitialized():
        raise ValueError("Uninitialized message.")

    # Attempt to unpack into known types
    for msg_type in known_message_types:
        msg_instance = msg_type()
        if packet.msg.Unpack(msg_instance):
            # Success!
            return msg_instance

    # It is not one of the known types: raise
    raise UnknownProtobufTypeError(packet.msg.TypeName())


def unpackb(
    b: bytes,
    *,
    known_message_types: Optional[Sequence[Type[Message]]] = None,
    max_packet_size: int = 65507,
) -> Message:
    """Unpack a Range Image Protocol packet from bytes.

    Args:
        b: Bytes containing a single Range Image Protocol packet.
        known_message_types: Sequence of protobuf message classes that are known and can be unpacked
            from the Packet.
        max_packet_size: Maximum allowed packet size in bytes. Max theoretical packet size is 4GB,
            given the 4-byte length field. Default max size is 65507, which is the max size of Range
            Image Protocol packet over UDP.

    Returns:
        The decoded protobuf message. One of the known_message_types.

    Raises:
        UnknownProtobufTypeError: If the message type in the Packet is not one of
            known_message_types.
        BadIDError: If the packet identifier is invalid.
        CRCMismatchError: If the packet CRC does not match.
        ExtraDataError: If there is extra data after parsing the packet.
        Exception: May raise other exceptions.
    """
    buf = io.BytesIO(b)
    try:
        msg = unpack(buf, known_message_types=known_message_types, max_packet_size=max_packet_size)
    except EOFError as e:
        raise ValueError("Incomplete Range Image Protocol packet.") from e
    if buf.tell() < len(b):
        # there is extra data after parsing the packet
        raise ExtraDataError(extra_data_length=len(b) - buf.tell())
    return msg
