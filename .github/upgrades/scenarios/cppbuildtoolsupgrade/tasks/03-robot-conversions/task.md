# 03-robot-conversions: Make Robot math and point indexes type-correct

Update the robot rotation calculations and axis-length constant to use explicit float-compatible expressions. Change the box drawing index tables from floating-point values to the unsigned index type expected by the OpenCV container API, preserving the existing twelve line connections.

## Research Findings

- The assessment reported nine warnings in `Robot.cpp`: six C4244 angle-calculation warnings, one C4305 axis-length warning, and two C4244 warnings caused by float index arrays passed to `std::vector::at`.
- `CRobot::createHT` receives `Vec3d`, so its rotation and translation components are double-based while the returned `Mat1f` is float-based; explicit casts preserve the existing float matrix representation.
- `createCoord` uses a float axis length, so the literal should be explicitly float-typed.
- `drawBox` uses twelve fixed integer vertex connections. The current arrays are floating-point even though they represent indexes; `std::size_t` matches the vector indexing API and preserves all values without conversion warnings.
- No project-file changes are needed.

**Done when**: `Robot.cpp` no longer emits its reported C4244, C4305, or C4244 point-index warnings and the same box edges are drawn.
