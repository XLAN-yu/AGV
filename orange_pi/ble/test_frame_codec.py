import unittest

from frame_codec import Assembler, encode


class FrameCodecTest(unittest.TestCase):
    def test_round_trip_multichunk(self) -> None:
        text = '{"type":"robot_status","payload":"' + "测" * 300 + '"}'
        frames = encode(text, mtu=64)
        self.assertGreater(len(frames), 1)
        assembler = Assembler()
        values = [assembler.accept(frame) for frame in frames]
        self.assertEqual(text, values[-1])
        self.assertTrue(all(value is None for value in values[:-1]))

    def test_missing_fragment_is_rejected(self) -> None:
        frames = encode("x" * 200, mtu=40)
        assembler = Assembler()
        assembler.accept(frames[0])
        with self.assertRaisesRegex(ValueError, "non-contiguous"):
            assembler.accept(frames[2])


if __name__ == "__main__":
    unittest.main()
