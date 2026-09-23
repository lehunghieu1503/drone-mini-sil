"""Visual link: runner -> external viewer over UDP (phase 1 of the visual plan).

Pure stdlib — no mujoco, no GL, no blocking. The SIL runner publishes the plant
pose as a small datagram; a separate viewer process (`tools/mujoco_view.py`)
renders it. The viewer is a pure consumer: if it is missing, late, or dead, the
SIL run is unaffected.

Design notes:
- Connected UDP (`connect` + `send`) so a closed port surfaces `ECONNREFUSED`;
  an unconnected `sendto` silently swallows it (verified on Linux).
- On error the sender backs off ~1 s then tries again (D9) — a viewer started
  late is picked up without restarting the run.
- Send rate is capped on the **wall clock** by `hz` (default 200) so the datagram
  rate stays bounded even when the sim runs far faster than realtime (D3).
"""

from __future__ import annotations

import socket
import struct
import time

POSE_FMT = "<Q7f"
POSE_SIZE = struct.calcsize(POSE_FMT)  # 36 bytes

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 45999
DEFAULT_HZ = 200.0
RETRY_S = 1.0

# Cosmetic z offset (D6 / RT-8): plants whose origin is the ground-contact point
# need half the body height so the mesh sits on the viewer floor; the MuJoCo
# plant's COM already rests at that height via contact, so it gets no offset.
_Z_OFFSET = {"mujoco": 0.0, "rk4": 0.0125, "rotorpy": 0.0125}
_DEFAULT_Z_OFFSET = 0.0125


def pack_pose(t_us: int, pos, quat) -> bytes:
    return struct.pack(
        POSE_FMT,
        int(t_us),
        float(pos[0]), float(pos[1]), float(pos[2]),
        float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]),
    )


def unpack_pose(buf: bytes):
    """Returns (t_us, pos3, quat4 wxyz). Raises ValueError on a bad datagram."""
    if len(buf) != POSE_SIZE:
        raise ValueError(f"bad pose size: {len(buf)} != {POSE_SIZE}")
    f = struct.unpack(POSE_FMT, buf)
    return f[0], (f[1], f[2], f[3]), (f[4], f[5], f[6], f[7])


def visual_pose(plant_name: str, pos, quat):
    """Cosmetic pose for the viewer: clamp to the floor, then per-plant offset."""
    offset = _Z_OFFSET.get(plant_name, _DEFAULT_Z_OFFSET)
    z = max(float(pos[2]), 0.0) + offset
    return (float(pos[0]), float(pos[1]), z), (
        float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))


class PoseSender:
    """Best-effort UDP publisher of the plant pose. Never raises from publish().

    Rate is capped on the wall clock (`hz`), not per simulated tick, so the
    datagram rate is bounded even when the sim runs far faster than realtime.
    """

    def __init__(self, port: int = DEFAULT_PORT, host: str = DEFAULT_HOST,
                 hz: float = DEFAULT_HZ, clock=time.monotonic, sock=None):
        self._clock = clock
        self._min_interval = (1.0 / hz) if hz > 0 else 0.0
        self._last_sent = None
        self._retry_at = 0.0
        self._closed = False
        self.sent = 0
        self.failed = 0
        self.attempts = 0
        if sock is None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setblocking(False)
            sock.connect((host, int(port)))  # enables ECONNREFUSED reporting
        self._sock = sock

    @property
    def healthy(self) -> bool:
        """True while no send error has been observed.

        UDP errors surface asynchronously, so this can be True for a few ms
        after the viewer disappears; treat it as observability, not liveness.
        """
        return self.sent > 0 and self.failed == 0

    def publish(self, t_us: int, plant_name: str, pos, quat) -> bool:
        """Send at most once per `1/hz` wall seconds. True if a datagram was sent."""
        if self._closed:
            return False
        now = self._clock()
        if now < self._retry_at:
            return False
        if self._last_sent is not None and (now - self._last_sent) < self._min_interval:
            return False
        vpos, vquat = visual_pose(plant_name, pos, quat)
        self.attempts += 1
        try:
            self._sock.send(pack_pose(t_us, vpos, vquat))
        except BlockingIOError:
            return False  # send buffer full: drop this frame, retry next tick
        except OSError:
            self.failed += 1
            self._retry_at = now + RETRY_S  # backoff, then try again (D9)
            return False
        self.failed = 0
        self.sent += 1
        self._last_sent = now
        return True

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._sock.close()
        except OSError:
            pass
