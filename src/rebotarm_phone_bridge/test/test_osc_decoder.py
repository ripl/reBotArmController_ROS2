import struct
import unittest

from rebotarm_phone_bridge.osc_decoder import decode_packet, OscDecodeError


def padded_string(value):
    data = value.encode("utf-8") + b"\0"
    return data + b"\0" * ((-len(data)) % 4)


def message(address, type_tags, *arguments):
    data = padded_string(address) + padded_string("," + type_tags)
    for type_tag, argument in zip(type_tags, arguments):
        if type_tag == "s":
            data += padded_string(argument)
        else:
            data += struct.pack({"i": ">i", "f": ">f", "d": ">d"}[type_tag], argument)
    return data


def bundle(*elements):
    data = b"#bundle\0" + struct.pack(">Q", 1)
    for element in elements:
        data += struct.pack(">I", len(element)) + element
    return data


class OscDecoderTests(unittest.TestCase):
    def test_decodes_phone_pose_bundle_in_order(self):
        packet = bundle(
            message("/phone/camera/session_id", "s", "session-123"),
            message("/phone/camera/position", "fff", 1.0, 2.0, 3.0),
            message("/phone/camera/rotation", "ffff", 0.0, 0.0, 0.0, 1.0),
        )

        decoded = decode_packet(packet)

        self.assertEqual(
            [item.address for item in decoded],
            [
                "/phone/camera/session_id",
                "/phone/camera/position",
                "/phone/camera/rotation",
            ],
        )
        self.assertEqual(decoded[0].arguments, ("session-123",))
        self.assertEqual(decoded[1].arguments, (1.0, 2.0, 3.0))

    def test_decodes_button_message(self):
        packet = message("/phone/input/button", "iiid", 42, 2, 1, 123.5)

        decoded = decode_packet(packet)

        self.assertEqual(decoded[0].arguments, (42, 2, 1, 123.5))

    def test_rejects_unsupported_type(self):
        packet = padded_string("/unsupported") + padded_string(",b")
        with self.assertRaises(OscDecodeError):
            decode_packet(packet)


if __name__ == "__main__":
    unittest.main()
