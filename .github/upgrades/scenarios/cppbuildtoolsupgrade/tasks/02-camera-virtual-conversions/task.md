# 02-camera-virtual-conversions: Make CameraVirtual conversions intentional and safe

Update the virtual-camera angle calculations and pixel-size constant to use explicit float-compatible expressions. In the vectorized point transformation, preserve `size_t` for standard-vector iteration and indexing, but provide the OpenCV matrix dimension as a checked, intentional `int` value because the relevant `Mat` constructor uses integer dimensions. Keep the matrix indexing types compatible with the OpenCV API and avoid changing transformation semantics.

## Research Findings

- The assessment reported 18 warnings in `CameraVirtual.cpp`: one C4305 for `_pixel_size`, C4244 warnings for the two angle-calculation groups, and C4267 warnings where `std::vector::size()`/`size_t` values reach OpenCV matrix dimensions or indexes.
- `std::vector<Mat>::size()` correctly returns `size_t`; the vector loops should continue using `size_t` for standard-container iteration.
- OpenCV `Mat` dimensions and the two-dimensional `Mat::at` overload use signed integer indices, so the conversion boundary should be explicit. The vector size will be checked against `int` capacity before creating the matrix, and loop indexes will be explicitly narrowed only at the OpenCV API boundary.
- `CV_PI` and `Vec3d` elements are double-based, while the destination math is intentionally float-based. Explicit casts preserve the existing float matrix representation without implicit narrowing warnings.
- The project configuration targets `v145`, uses OpenCV headers from `..\opencv\include`, and needs no project-file changes.

**Done when**: `CameraVirtual.cpp` no longer emits its reported C4305, C4244, or C4267 warnings and the vectorized transform still constructs and indexes the same 4-by-N matrix correctly.
