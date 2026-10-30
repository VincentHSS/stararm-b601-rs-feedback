"""原生 SDK + 伪终端协议测试：只访问 PTY，不接触真实串口。"""
import os
import select
import struct
import threading
import unittest

try:
    from motorbridge_smart_servo import FashionStarServo
    PATCHED = hasattr(FashionStarServo, 'set_zero_confirm_samples')
except ImportError:
    PATCHED = False


@unittest.skipUnless(PATCHED, 'requires bundled motorbridge-smart-servo rsfix1 SDK')
class SmartServoTransportTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = os.openpty()
        self.angle = 0
        self.missing = set()
        self.corrupt = set()
        self.done = threading.Event()
        self.commands = []
        self.bus = FashionStarServo(os.ttyname(self.slave))
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.bus.close()
        self.done.set()
        self.thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)
        self.assertTrue(all(code == 25 for code in self.commands), 'only monitor queries allowed')

    def serve(self):
        pending = b''
        while not self.done.is_set():
            ready, _, _ = select.select([self.master], [], [], .02)
            if not ready:
                continue
            pending += os.read(self.master, 1024)
            while len(pending) >= 5:
                size = pending[3]+5
                if len(pending) < size:
                    break
                request, pending = pending[:size], pending[size:]
                code = request[2]
                self.commands.append(code)
                if code != 25:
                    continue
                response = b''
                for i in request[7:-1]:
                    if i in self.missing:
                        continue
                    angle = self.angle if i == 1 else 100
                    params = struct.pack('<BHHHHBih', i, 12000, 30, 360, 250, 0, angle, 0)
                    packet = bytes([5, 28, 22, len(params)])+params
                    response += packet+bytes([(sum(packet)+(i in self.corrupt)) & 255])
                if response:
                    os.write(self.master, response)

    def test_stationary_zero_is_fresh_for_90_sync_reads(self):
        for _ in range(90):
            data = self.bus.sync_monitor(list(range(7)))
            self.assertTrue(all(data[i].reliable for i in range(7)))
            self.assertEqual(data[1].angle_deg, 0)

    def test_missing_servo_is_unreliable_even_after_accepted_zero(self):
        self.assertTrue(self.bus.sync_monitor([1])[1].reliable)
        self.missing.add(1)
        data = self.bus.sync_monitor(list(range(7)))
        self.assertFalse(data[1].reliable)
        self.assertEqual(data[1].angle_deg, 0)
        self.assertTrue(all(data[i].reliable for i in range(7) if i != 1))

    def test_corrupt_zero_frame_is_not_accepted_as_fresh(self):
        self.assertTrue(self.bus.sync_monitor([1])[1].reliable)
        self.corrupt.add(1)
        self.assertFalse(self.bus.sync_monitor(list(range(7)))[1].reliable)

    def test_zero_jump_confirms_in_three_fresh_frames_then_stays_valid(self):
        self.assertEqual(self.bus.set_zero_confirm_samples(3), 30)
        self.angle = 100
        self.assertEqual(self.bus.sync_monitor([1])[1].angle_deg, 10)
        self.angle = 0
        for _ in range(2):
            data = self.bus.sync_monitor([1])[1]
            self.assertFalse(data.reliable)
            self.assertEqual(data.angle_deg, 10)
        for _ in range(5):
            data = self.bus.sync_monitor([1])[1]
            self.assertTrue(data.reliable)
            self.assertEqual(data.angle_deg, 0)
