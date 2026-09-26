# 05: Calibration (camera pixels to RoArm mm)

**Done when:** `homography` is saved in `config.json`, and the pointing test below misses by 15 mm or less everywhere on the bin floor.

## How it works

The RoArm moves a tag on its gripper to known `x, y` points at `pick_z`. The SO-101 camera at `look` sees where the tag appears in the image. From at least 4 of those (pixel ↔ mm) pairs, OpenCV fits a homography: a mapping from a pixel to a point on the bin-floor plane. After that, any pixel the vision model returns converts straight to a RoArm target (`to_arm()`).

## Tasks

- [ ] Print AprilTag 36h11 **id 0** (`calib_tag`) about 30 mm wide, from `AprilTags/print.pdf`. Leave a white border around it.
- [ ] Tape the tag flat and face-up on the gripper, **as close to the fingertips as you can**, where the camera at `look` can see it.
- [ ] Set `calib_points` to cover the **whole bin floor**, corners included. Use at least 5 points; a 3×3 grid (9) is better. All of them must be inside `reach_mm` and visible at `look`.
- [ ] Clear the bin, then run:
  ```sh
  python main.py calibrate
  ```
  It prints the pixel it found for each point. If a point says `tag not seen`, move it inward or change the `look` pose (and re-save it).
- [ ] **Pointing test:** put 5 small items (bottle caps) spread over the floor. Take a photo at `look`, click or read each cap's pixel from the photo, and convert it with `to_arm`. Then `goto` that `x, y` at `pick_z + 20` and measure the miss with a ruler.
  - Misses above 15 mm, all in the same direction: the tag is too far from the fingertips. Move it, or measure the offset and add it to the x/y the RoArm is sent.
  - Misses that grow toward the edges: add more `calib_points` near the edges.
  - Random misses: the `look` pose isn't repeatable (back to step 03).
- [ ] Take the tag off the gripper, or cover it, so the vision model doesn't label it as trash.

## Keep it valid

The homography goes stale as soon as the bin, the RoArm base, the SO-101 base, the camera mount or the `look` pose moves. Optional cheap check: tape one tag (not id 0) on the bin rim and note its pixel. If it has moved more than about 5 px, re-run `calibrate` (about 2 minutes).
