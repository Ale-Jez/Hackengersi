# 06: Pick and sort

**Done when:** `python main.py sort` clears a bin with 6 mixed items (2 per category) in at most `max_rounds` rounds, with at least 5 of 6 in the right container, three times in a row.

## The loop (already in `main.py`)

`sort_can()` runs up to `max_rounds` rounds. Each round:

1. RoArm to `park`, out of the picture.
2. SO-101 to `look`, take a photo, SO-101 to `stow`.
3. Send the photo to Brev and get back `[{label, bin, x, y}]`. The annotated photo is saved as `last_look.jpg`.
4. For each item: convert the pixel to mm with `to_arm`, skip it if it's outside `reach_mm`, then `pick()`:
   above the item, down to `pick_z`, close the gripper, back up, over the right container, open the gripper.
5. Take a new photo next round (items shift). Stop when nothing is left or nothing could be picked.

## Tasks, in order

- [ ] **One item, one category.** Put a can in the middle of the bin and run `python main.py sort`. Watch `last_look.jpg` and the printed mm.
- [ ] **Tune by category.** Try one of each. Typical fixes:
  - A standing bottle is grabbed off-centre: vision step 04, task 3 (ask for the point touching the floor).
  - A can slips: `grip_closed` isn't tight enough, or the jaws hit the can end-on (see "Orientation" below).
  - Paper or a bag is missed: lower `pick_z` a few mm. Soft items need the fingers almost on the floor.
  - The gripper hits the floor: raise `pick_z`, or the shoulder sag is in the way (step 02 pitfalls).
- [ ] **Spread out:** items in the corners and at the edges of the reach.
- [ ] **Clutter:** 6 mixed items, some touching.
- [ ] Check the arm's temperature and the voltage after each 10-pick session.

## Add only if a test shows you need it

- **Pick height per category:** make `pick_z` a dictionary by bin. Add it only if a single value can't serve both cans and paper.
- **Orientation:** have the model also return an angle for long items and send it as `r` (wrist roll) in `goto()`. Add it only if lying bottles keep slipping.
- **Missed-grasp check:** after `gripper(grip_closed)`, `where()["g"]` close to `grip_closed` means the fingers closed on nothing. Skip the container move and let the next round retry. (`g` feedback has been real since the servo swap.)
- **Depth for tall items:** estimate height from bounding-box size. Only if the floor-contact prompt isn't enough.
