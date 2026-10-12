# C++ Build Tools Upgrade Plan

## Overview

**Target**: Remove the 29 conversion warnings reported by the `v145` MSVC toolset rebuild in the single `7825_Template.vcxproj` project.
**Scope**: Small C++ project; source-only warning fixes in `CameraReal.cpp`, `CameraVirtual.cpp`, and `Robot.cpp`. The `.vcxproj` configuration already targets `v145` and does not require changes.

## Tasks

### 01-camera-real-conversions: Make CameraReal floating-point constants explicit

Update the ChArUco board dimensions in `CameraReal.cpp` so constants assigned to `float` variables have an explicit float type. This removes the two C4305 truncation warnings without changing the intended metric values or camera behavior.

**Done when**: `CameraReal.cpp` no longer emits the two reported C4305 warnings and its values remain equivalent to the current code.

---

### 02-camera-virtual-conversions: Make CameraVirtual conversions intentional and safe

Update the virtual-camera angle calculations and pixel-size constant to use explicit float-compatible expressions. In the vectorized point transformation, preserve `size_t` for standard-vector iteration and indexing, but provide the OpenCV matrix dimension as a checked, intentional `int` value because the relevant `Mat` constructor uses integer dimensions. Keep the matrix indexing types compatible with the OpenCV API and avoid changing transformation semantics.

**Done when**: `CameraVirtual.cpp` no longer emits its reported C4305, C4244, or C4267 warnings and the vectorized transform still constructs and indexes the same 4-by-N matrix correctly.

---

### 03-robot-conversions: Make Robot math and point indexes type-correct

Update the robot rotation calculations and axis-length constant to use explicit float-compatible expressions. Change the box drawing index tables from floating-point values to the unsigned index type expected by the OpenCV container API, preserving the existing twelve line connections.

**Done when**: `Robot.cpp` no longer emits its reported C4244, C4305, or C4244 point-index warnings and the same box edges are drawn.

---

### 04-build-validation: Rebuild and verify the complete solution

Run an incremental build after the source fixes, address any warnings or errors introduced by the changes, and then perform the required final full rebuild. Compare the final result against the assessment so no out-of-scope regressions or new warnings remain.

**Done when**: The final full rebuild of `7825_RoboticsLab3.sln` completes with zero errors and zero warnings, or any remaining issue is demonstrated to be external and explicitly reported.
