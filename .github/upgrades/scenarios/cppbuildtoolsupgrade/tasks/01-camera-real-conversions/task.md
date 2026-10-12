# 01-camera-real-conversions: Make CameraReal floating-point constants explicit

Update the ChArUco board dimensions in `CameraReal.cpp` so constants assigned to `float` variables have an explicit float type. This removes the two C4305 truncation warnings without changing the intended metric values or camera behavior.

## Research Findings

- A full `v145` rebuild reported two C4305 warnings in `CameraReal.cpp`, at the `size_square` and `size_mark` initializers inside `CCameraReal::createChArUcoBoard`.
- The project is `O:\CPlus\Fall2026_ELEX7825_Robotics\Robotics_Labs\7825_RoboticsLab3\7825_Template.vcxproj`, a single x64 application using OpenCV headers from `..\opencv\include`.
- The existing values are metric constants `0.04` and `0.02`; adding the `f` suffix preserves their intended float values while removing implicit double-to-float conversion.
- No project-file changes are needed.

**Done when**: `CameraReal.cpp` no longer emits the two reported C4305 warnings and its values remain equivalent to the current code.
