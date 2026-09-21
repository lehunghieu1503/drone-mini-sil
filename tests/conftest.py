import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from _bind import load_lib  # noqa: E402


@pytest.fixture(scope="session")
def lib():
    return load_lib()
