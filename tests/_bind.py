"""Host-lib / sim build + ctypes binding helpers (shared by tests)."""

from __future__ import annotations

import ctypes
import pathlib
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _cmake_build(src: pathlib.Path, build_dir: pathlib.Path, args=None, force=False):
    build_dir.mkdir(parents=True, exist_ok=True)
    cfg = ["cmake", "-S", str(src), "-B", str(build_dir)]
    if args:
        cfg += args
    if force or not (build_dir / "CMakeCache.txt").exists():
        subprocess.check_call(cfg)
    subprocess.check_call(["cmake", "--build", str(build_dir), "-j"])
    return build_dir


def build_host(force: bool = False) -> pathlib.Path:
    return _cmake_build(ROOT / "firmware", ROOT / "build" / "host",
                        ["-DDRONE_HOST_LIB=ON"], force=force)


def build_sil(force: bool = False) -> pathlib.Path:
    return _cmake_build(ROOT / "sim", ROOT / "build" / "sim", force=force)


def load_lib() -> ctypes.CDLL:
    so = build_host() / "libdrone_flight.so"
    if not so.exists():
        build_host(force=True)
    return ctypes.CDLL(str(so))


def bind(lib, name, restype, argtypes=None):
    fn = getattr(lib, name)
    fn.restype = restype
    fn.argtypes = argtypes or []
    return fn


def sil_runner_path() -> pathlib.Path:
    return build_sil() / "sil_runner"
