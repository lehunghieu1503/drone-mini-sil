"""Phase 3 — viewer process helpers. Headless: no display, no GL."""

import importlib
import os
import pathlib
import socket
import subprocess
import sys

import pytest

from plant.visual_link import pack_pose


def _view():
    return importlib.import_module("tools.mujoco_view")


def _pair():
    rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    tx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    rx.bind("")
    tx.connect(rx.getsockname())
    rx.setblocking(False)
    return rx, tx


def test_drain_latest_returns_newest_and_drains():
    view = _view()
    rx, tx = _pair()
    try:
        for i in range(3):
            tx.send(pack_pose(i, (i, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
        pose = view.drain_latest(rx)
        assert pose is not None
        assert pose[0] == 2  # newest t_us
        assert pose[1][0] == pytest.approx(2.0)
        assert view.drain_latest(rx) is None  # fully drained
    finally:
        rx.close()
        tx.close()


def test_drain_latest_ignores_malformed_and_empty():
    view = _view()
    rx, tx = _pair()
    try:
        assert view.drain_latest(rx) is None  # nothing queued
        tx.send(b"\x00" * 10)  # malformed
        tx.send(pack_pose(7, (1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0)))
        tx.send(b"garbage")
        pose = view.drain_latest(rx)
        assert pose is not None and pose[0] == 7  # garbage never clears a good pose
    finally:
        rx.close()
        tx.close()


def test_module_import_does_not_require_mujoco():
    """Check in a clean interpreter (the suite imports mujoco for other tests)."""
    root = pathlib.Path(__file__).resolve().parents[1]
    code = ("import tools.mujoco_view, sys; "
            "assert 'mujoco' not in sys.modules, 'viewer pulled in mujoco at import'; "
            "print('ok')")
    out = subprocess.run([sys.executable, "-c", code], cwd=root, text=True,
                         capture_output=True, env={**os.environ, "PYTHONPATH": "."})
    assert out.returncode == 0, out.stderr


def test_view_parser_defaults_and_flags():
    view = _view()
    defaults = view.build_parser().parse_args([])
    assert defaults.port == view.DEFAULT_PORT
    assert defaults.fps == 60.0
    assert defaults.follow is True
    assert view.build_parser().parse_args(["--no-follow", "--fps", "30"]).follow is False
    assert view.build_parser().parse_args(["--port", "12345"]).port == 12345


@pytest.mark.parametrize("flag,value", [
    ("--port", "0"),
    ("--port", "70000"),
    ("--fps", "0"),
    ("--fps", "-1"),
    ("--fps", "inf"),
    ("--fps", "nan"),
])
def test_view_parser_rejects_bad_values(flag, value):
    view = _view()
    with pytest.raises(SystemExit):
        view.build_parser().parse_args([flag, value])


def test_display_ok_false_without_display():
    """GLFW cannot initialise without a display -> the viewer must report it."""
    pytest.importorskip("glfw")
    root = pathlib.Path(__file__).resolve().parents[1]
    env = {k: v for k, v in os.environ.items()
           if k not in ("DISPLAY", "WAYLAND_DISPLAY")}
    env["PYTHONPATH"] = "."
    out = subprocess.run(
        [sys.executable, "-c", "import tools.mujoco_view as v; print(v.display_ok())"],
        cwd=root, text=True, capture_output=True, env=env)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip() == "False"
