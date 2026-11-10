# Lab 7 (Trajectories): things to discuss before starting

Notes from reading `Labpdfs/Lab7-Trajectories.pdf` against the code as it stands after Lab 6.
Nothing here has been implemented.

## Decisions for you

1. **Code layout.** Same choice as Lab 6: new `*_lab7` functions that reuse Lab 5/6, or extend the
   Lab 6 functions? The Lab 6 settings window is nearly full (it reaches the bottom of the
   720 px canvas), so a Lab 7 "trajectory mode" option needs either a tighter layout or a second window.
2. **`jtraj` shape.** The lab wants initial/final position, initial/final velocity, zero
   acceleration at both ends, and a step count, with the output stored in a vector. That is a
   quintic (5th order) polynomial. Should it work on one value at a time
   (`vector<double> jtraj(q0, q1, v0, v1, steps)`) or on all four joints at once
   (`vector<Vec4d>`)? One value at a time is easier to understand and is reused unchanged for
   X, Y, θ in Part C.
3. **Which marker IDs for Part C-b?** Part C-a uses ID 50. The box sheet has three cubes, but the
   lab does not say the other two IDs (51 and 52 is my guess). Worth checking the printout.
4. **Does Z get interpolated?** Part C-a says "interpolate values for X, Y, θ". Z is not listed.
   Hold Z at its current value, or interpolate it too?
5. **Fixed duration or distance-scaled speed** for Part C-b? The lab allows either.

## Things in the current code that Lab 7 will run into

- **Marker detection handles one marker.** `detect_single_marker()` runs the detector and returns
  one ID. For three markers it should detect once per frame and return every ID found, otherwise
  the detector runs three times per frame.
- **Two webcam frames are grabbed per drawn frame.** `detect_markers()` (Lab 4) calls
  `get_image()` itself, so the board pose comes from a different frame than the one displayed and
  the one the marker is found in. It works, but it halves the frame rate and the board and marker
  poses are slightly out of step when the camera moves. Passing the image in would fix it; that
  means touching Lab 4 code, so it is your call.
- **Marker pose is noisy.** In the synthetic test a 21 mm marker at 0.5 m was off by about 4 mm in
  X and 6 mm in Z, and it will jitter frame to frame with a real camera. Lab 7's smooth
  trajectories will look jerky if the target jitters. Options: average the last few poses, or only
  re-plan when the marker has moved more than a few mm.
- **θ wrap-around.** Interpolating θ from 170° to −170° the naive way goes the long way round
  (340°) instead of 20°. A Cartesian trajectory needs to take the short way.
- **Elbow choice during a move.** `ikine()` takes `elbow_flip` as an input. A trajectory should
  keep it fixed for the whole move, or the arm will snap between solutions.
- **Unreachable targets mid-trajectory.** A straight line between two reachable points always
  stays inside the 300 mm circle, so this only matters if a marker is moved out of reach. Lab 6
  holds the last pose; Lab 7 needs to decide whether to pause, skip that marker, or stop.
- **Int joint sliders.** Lab 6 keeps exact joints in `_joints_lab6` and only displays rounded
  values on the sliders. `jtraj` output should go there too, not into the int `_joint[]`.
- **Time base.** Animations advance one step per drawn frame (`waitKey(10)` plus drawing time),
  so "number of time steps" is really "number of frames". With the webcam on, frames are slower,
  so the same trajectory takes longer on the board than on the virtual camera.
- **Part B joint target `(-180, 90, 90, 15 cm)`** is within all slider ranges (j3 max is 150 mm),
  so no range changes are needed.

## To verify with real hardware first (carried over from Lab 6)

- `MARKER_SIZE` (21 mm) and `MARKER_BOX_SIZE` (30 mm) in Robot.cpp were measured off the PDF, not
  a printout.
- Marker 50 is assumed to be from `DICT_6X6_250`, the same dictionary as the board.
- Lab 6 Part B has only been run against a synthetic image, never a real webcam.
