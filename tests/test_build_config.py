"""T1.12/T1.13 — C++20 build policy and .hpp/.cpp-only flight core."""

import pathlib

from _bind import ROOT

FLIGHT = ROOT / "firmware" / "main" / "flight"


def test_t1_13_cxx20_and_rt_flags():
    top = (ROOT / "firmware" / "CMakeLists.txt").read_text()
    flight = (FLIGHT / "CMakeLists.txt").read_text()
    combined = top + flight
    assert "cxx_std_20" in combined or "CMAKE_CXX_STANDARD 20" in combined
    assert "-fno-exceptions" in flight
    assert "-fno-rtti" in flight

    main_cmake = (ROOT / "firmware" / "main" / "CMakeLists.txt").read_text()
    assert "-std=gnu++20" in main_cmake  # IDF branch pin

    sdk = (ROOT / "firmware" / "sdkconfig.defaults").read_text()
    assert "CONFIG_COMPILER_CXX_EXCEPTIONS=n" in sdk
    assert "CONFIG_COMPILER_CXX_RTTI=n" in sdk


def test_t1_12_flight_only_hpp_cpp():
    bad = [p.name for p in FLIGHT.iterdir() if p.suffix in (".c", ".h")]
    assert bad == [], f"flight/ must contain only .hpp/.cpp, found {bad}"


def test_t1_3_no_idf_includes_in_flight():
    import re

    pattern = re.compile(r'#\s*include\s*[<"](esp_[^>"]*|driver/[^>"]*|freertos/[^>"]*|sdkconfig\.h)')
    offenders = []
    for path in list(FLIGHT.glob("*.hpp")) + list(FLIGHT.glob("*.cpp")):
        for i, line in enumerate(path.read_text().splitlines(), 1):
            if pattern.search(line):
                offenders.append(f"{path.name}:{i}")
    assert offenders == [], f"ESP-IDF include in flight/: {offenders}"


def test_t1_14_hal_hw_in_idf_srcs():
    """The chip component must compile hal_hw.cpp so the #error guard can fire."""
    cmake = (ROOT / "firmware" / "main" / "CMakeLists.txt").read_text()
    assert "hal/hal_hw.cpp" in cmake
    hal = (ROOT / "firmware" / "main" / "hal" / "hal_hw.cpp").read_text()
    assert "return false" in hal  # init() stays fail-closed until P9
