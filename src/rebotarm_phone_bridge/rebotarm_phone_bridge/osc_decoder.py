from __future__ import annotations

from dataclasses import dataclass
import struct


class OscDecodeError(ValueError):
    pass


@dataclass(frozen=True)
class OscMessage:
    address: str
    arguments: tuple[object, ...]


def _read_string(data: bytes, offset: int) -> tuple[str, int]:
    try:
        terminator = data.index(0, offset)
        value = data[offset:terminator].decode("utf-8")
    except (UnicodeDecodeError, ValueError) as exc:
        raise OscDecodeError("invalid OSC string") from exc
    next_offset = (terminator + 4) & ~3
    if next_offset > len(data):
        raise OscDecodeError("truncated OSC string padding")
    return value, next_offset


def _read_value(data: bytes, offset: int, type_tag: str) -> tuple[object, int]:
    formats = {"i": ">i", "f": ">f", "d": ">d"}
    if type_tag == "s":
        return _read_string(data, offset)
    if type_tag not in formats:
        raise OscDecodeError(f"unsupported OSC type tag {type_tag!r}")
    size = struct.calcsize(formats[type_tag])
    if offset + size > len(data):
        raise OscDecodeError("truncated OSC argument")
    return struct.unpack_from(formats[type_tag], data, offset)[0], offset + size


def _decode_message(data: bytes) -> OscMessage:
    address, offset = _read_string(data, 0)
    if not address.startswith("/"):
        raise OscDecodeError("OSC address must start with '/'")
    type_tags, offset = _read_string(data, offset)
    if not type_tags.startswith(","):
        raise OscDecodeError("OSC type tag string must start with ','")
    arguments = []
    for type_tag in type_tags[1:]:
        value, offset = _read_value(data, offset, type_tag)
        arguments.append(value)
    return OscMessage(address, tuple(arguments))


def decode_packet(data: bytes) -> tuple[OscMessage, ...]:
    if not data:
        raise OscDecodeError("empty OSC packet")
    if not data.startswith(b"#bundle\x00"):
        return (_decode_message(data),)
    if len(data) < 16:
        raise OscDecodeError("truncated OSC bundle")

    messages: list[OscMessage] = []
    offset = 16
    while offset < len(data):
        if offset + 4 > len(data):
            raise OscDecodeError("truncated OSC bundle element size")
        element_size = struct.unpack_from(">I", data, offset)[0]
        offset += 4
        end = offset + element_size
        if element_size == 0 or end > len(data):
            raise OscDecodeError("invalid OSC bundle element size")
        messages.extend(decode_packet(data[offset:end]))
        offset = end
    return tuple(messages)
