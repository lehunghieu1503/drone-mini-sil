#include "hal/hal_mock.hpp"

// MockHal is header-only by design; this TU keeps the target list stable so the
// host lib and CMake source lists do not diverge.
namespace drone {
namespace {
[[maybe_unused]] const int kMockHalTuAnchor = 0;
}
}  // namespace drone
