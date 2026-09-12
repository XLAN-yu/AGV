import unittest

from pair_event import PAIR_EVENT_PACKET, PairEventProtocol


class PairEventProtocolTest(unittest.TestCase):
    def test_only_exact_versioned_packet_triggers(self) -> None:
        requests: list[bool] = []
        protocol = PairEventProtocol(lambda: requests.append(True))

        protocol.datagram_received(b"PAIR\n", ("127.0.0.1", 1))
        protocol.datagram_received(PAIR_EVENT_PACKET + b"extra", ("127.0.0.1", 1))
        protocol.datagram_received(PAIR_EVENT_PACKET, ("127.0.0.1", 1))

        self.assertEqual(requests, [True])
