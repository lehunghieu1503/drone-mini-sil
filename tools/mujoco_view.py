"""MuJoCo live viewer for the SIL runner (visual plan phase 3).

Runs as a **separate process**: receives plant pose datagrams published by
`plant/runner.py --visual` and renders them with the passive MuJoCo viewer.
Isolation is deliberate — a GL crash, a freeze, or a missing display here can
never affect the SIL run or `make gate`.

Usage (two terminals):
    make view      # terminal 1
    make visual    # terminal 2

or directly:
    PYTHONPATH=. .venv/bin/python tools/mujoco_view.py --port 45999 --fps 60
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import socket
import sys
import time
import warnings

from plant.visual_link import DEFAULT_PORT, unpack_pose

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCENE_XML = ROOT / "plant" / "assets" / "bitcraze_crazyflie_2" / "scene.xml"
EXIT_CONFIG = 5
_RECV_MAX = 2048
_TEARDOWN_GRACE_S = 0.2


def display_ok() -> bool:
    """True if GLFW can initialise (a usable display exists).

    MuJoCo's viewer exits the process natively when GLFW cannot start, so this
    pre-flight is the only way to report the failure as a normal exit code.
    """
    import glfw

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")  # glfw warns loudly when there is no display
        ok = bool(glfw.init())
    glfw.terminate()
    return ok


def drain_latest(sock: socket.socket):
    """Return the newest decoded pose from queued datagrams, or None.

    Latest-wins: stale frames are dropped so the viewer always renders the most
    recent physics state regardless of how fast the SIL run is going.
    Malformed datagrams are ignored (they never clear a good pose).
    """
    latest = None
    while True:
        try:
            buf = sock.recv(_RECV_MAX)
        except (BlockingIOError, InterruptedError):
            break
        except OSError:
            break
        try:
            latest = unpack_pose(buf)
        except ValueError:
            continue
    return latest


def _teardown(viewer, sock: socket.socket, code: int) -> None:
    """Close the socket/viewer, give the render thread a beat, then hard-exit.

    `os._exit` skips atexit so the native GL teardown (which can crash with
    SIGSEGV/SIGABRT/X errors) never runs (RT-1). The viewer is its own process,
    so even a crash here cannot affect the SIL run.
    """
    try:
        sock.close()
    except OSError:
        pass
    try:
        viewer.close()
    except Exception:  # noqa: BLE001 - teardown must not mask the exit code
        pass
    # Handle.is_running() flips false immediately on close(), so a fixed grace
    # period is the only way to let the render thread finish its current frame.
    time.sleep(_TEARDOWN_GRACE_S)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


def _port_type(text: str) -> int:
    value = int(text)
    if not (1 <= value <= 65535):
        raise argparse.ArgumentTypeError("port must be in 1..65535")
    return value


def _fps_type(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or not (0.0 < value <= 1000.0):
        raise argparse.ArgumentTypeError("fps must be a finite number in (0, 1000]")
    return value


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="mujoco_view")
    ap.add_argument("--port", type=_port_type, default=DEFAULT_PORT,
                    help="UDP port to listen on (default 45999)")
    ap.add_argument("--fps", type=_fps_type, default=60.0, help="render rate cap")
    ap.add_argument("--follow", action=argparse.BooleanOptionalAction, default=True,
                    help="keep the camera looking at the drone (default: on)")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    try:
        import mujoco
        import mujoco.viewer  # noqa: F401 - submodule is not auto-imported
    except ImportError as exc:
        sys.stderr.write(f"mujoco_view: {exc}. Run `make venv-visual`.\n")
        return EXIT_CONFIG

    try:
        if not display_ok():
            sys.stderr.write(
                "mujoco_view: no usable display (GLFW could not initialise).\n"
                "             Run on a machine with a GUI; on macOS run via `mjpython`.\n")
            return EXIT_CONFIG
    except ImportError as exc:
        sys.stderr.write(f"mujoco_view: {exc}. Run `make venv-visual`.\n")
        return EXIT_CONFIG

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", args.port))  # no SO_REUSEADDR: a second viewer must fail loudly
    except (OSError, OverflowError, ValueError) as exc:
        sys.stderr.write(
            f"mujoco_view: cannot bind 127.0.0.1:{args.port} ({exc}).\n"
            "             Another viewer running? Use --port.\n")
        sock.close()
        return EXIT_CONFIG
    sock.setblocking(False)

    try:
        model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
        data = mujoco.MjData(model)
    except Exception as exc:  # noqa: BLE001 - report and exit cleanly
        sys.stderr.write(f"mujoco_view: cannot load {SCENE_XML.name} ({exc})\n")
        return EXIT_CONFIG

    mujoco.mj_forward(model, data)
    try:
        viewer = mujoco.viewer.launch_passive(model, data)
    except (RuntimeError, mujoco.FatalError) as exc:
        sys.stderr.write(
            f"mujoco_view: cannot open a window ({exc}).\n"
            "             Needs a working display; on macOS run via `mjpython`.\n")
        return EXIT_CONFIG

    period = 1.0 / args.fps
    print(f"[mujoco_view] listening on udp 127.0.0.1:{args.port} "
          f"(fps={args.fps:g}, follow={args.follow}) — Ctrl-C or close the window to quit",
          flush=True)

    code = 0
    got_first = False
    try:
        while viewer.is_running():
            frame_start = time.perf_counter()
            pose = drain_latest(sock)
            if pose is not None:
                _t_us, pos, quat = pose
                if not got_first:
                    got_first = True
                    print(f"[mujoco_view] first pose received at t={_t_us / 1e6:.2f}s", flush=True)
                data.qpos[0:3] = pos          # renderer reads its own shadow copy
                data.qpos[3:7] = quat
                if args.follow:
                    # cam is shared with the render thread: mutate under its lock.
                    # Never nest sync() inside lock() — sync locks internally.
                    with viewer.lock():
                        viewer.cam.lookat[:] = pos
                viewer.sync(state_only=True)
            remaining = period - (time.perf_counter() - frame_start)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - keep the viewer exiting cleanly
        sys.stderr.write(f"mujoco_view: {type(exc).__name__}: {exc}\n")
        code = 1
    finally:
        _teardown(viewer, sock, code)
    return code


if __name__ == "__main__":
    sys.exit(main())
