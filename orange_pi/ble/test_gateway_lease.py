import asyncio
import unittest

from gateway_lease import GatewayLease


class FakeSocket:
    def __init__(self) -> None:
        self.sent: list[str] = []
        self.closed = False
        self.incoming: asyncio.Queue[str | None] = asyncio.Queue()

    def __aiter__(self):
        return self

    async def __anext__(self):
        value = await self.incoming.get()
        if value is None:
            raise StopAsyncIteration
        return value

    async def send(self, raw: str) -> None:
        self.sent.append(raw)

    async def close(self, **_: object) -> None:
        self.closed = True


class GatewayLeaseTest(unittest.IsolatedAsyncioTestCase):
    async def test_claim_forward_status_and_expiry(self) -> None:
        socket = FakeSocket()
        notified: list[str] = []

        async def connector(*_: object, **__: object) -> FakeSocket:
            return socket

        async def notify(raw: str) -> None:
            notified.append(raw)

        lease = GatewayLease("ws://gateway/ws", "http://rover", notify, 0.1, connector)
        await lease.forward('{"_ble":"claim"}')
        command = '{"type":"drive","linear":0,"angular":0,"seq":1}'
        await lease.forward(command)
        self.assertEqual([command], socket.sent)
        await socket.incoming.put('{"type":"robot_status","event":"telemetry"}')
        await asyncio.sleep(0)
        self.assertEqual(1, len(notified))
        await asyncio.sleep(0.12)
        self.assertTrue(await lease.release_if_expired())
        self.assertTrue(socket.closed)
        self.assertIsNone(lease.ws)

    async def test_command_without_claim_is_rejected(self) -> None:
        async def connector(*_: object, **__: object) -> FakeSocket:
            return FakeSocket()
        lease = GatewayLease("ws://gateway/ws", "http://rover", lambda _: asyncio.sleep(0), connector=connector)
        with self.assertRaisesRegex(RuntimeError, "not active"):
            await lease.forward('{"type":"estop","seq":1}')


if __name__ == "__main__":
    unittest.main()
