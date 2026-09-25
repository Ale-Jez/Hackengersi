# 06: Demo and Submission

## Demo run (3 minutes, adjust to the slot)

Three people: **Narrator** (R4), **Operator** (R1, gamepad E-STOP for CubeBot), **Station keeper** (R3, hand on the arms' power strip). R2 watches the dashboard and flips fallbacks.

| Time | Robots / screen | Narrator says |
|---|---|---|
| 0:00-0:20 | Arena with containers on the floor; big screen shows **0,00 zł** | Hold up a bag of empties: "Every Polish kitchen has one of these. Each bottle is 50 groszy, but only if someone collects it and takes it back." |
| 0:20-0:35 | Point to CubeBot, then to the dock with both arms | "This is a return machine on legs. One robot to collect from anywhere on the floor, two arms to check and bag in one known spot." |
| 0:35-1:40 | Press START. CubeBot searches, walks to a bottle, pushes it into the funnel, backs off. SO-101 rolls it, **beep**, the EAN and product name appear, ACCEPT. RoArm bags it. Counter **+0,50 zł** with a ding | Narrate what the dashboard shows: sees the bottle, steers, finds the dock, hands off, reads the barcode, checks the deposit list, bags. Say "autonomous" only in `auto` mode |
| 1:40-2:10 | CubeBot brings the jar (or a non-deposit item). Scan → **REJECT** → reject bin, buzz | "Not every container has a deposit. Like a real machine, it checks the barcode and rejects this one." |
| 2:10-2:30 | Another bottle or a can if Tier 2 is stable | Counter 1,00 zł |
| 2:30-2:45 | Press **Voucher**: `POST /transaction` → voucher QR + amount on screen | "The station follows the Kaucja.pl OpenAPI flow: one transaction, one voucher. Take the bag and the voucher to the shop." |
| 2:45-3:00 | Slide: what's next | Real API access, bag sealing (`/bag-replacement`), more container types (only claims we can back) |

**Pre-run checklist (60 s):** batteries fresh, both arms at `home`, both E-STOPs tested, `demo.yaml` loaded, mock DRS running, dashboard on the big screen, lamp on and camera focus locked, dock taped in its marks and empty, bag and reject bin empty, counter at 0, containers on their floor marks.

**If it breaks live:** don't apologise or improvise. Say "switching to operator assist" and step down one rung of the right ladder in `05`. A prepared, calm fallback scores better than a silent crash. If everything is down, play the backup video.

## Honest framing (pitch and README)

- **Live:** container detection, dock finding, walking, pushing, the handoff between three robots, barcode reading, accept/reject, counter.
- **Taught:** both arms' poses, recorded once by hand and replayed. The dock is designed so that this is enough.
- **Mocked:** the Kaucja.pl server. Our client follows the official OpenAPI quick start (`POST /transaction`, vouchers, bag replacement); the API is open only to registered shops, so today it talks to our mock. The deposit list is local, like a shop till's.
- **Prior work, declared:** CubeBot (leg IK, gait tables, Hailo pipeline), RoArm firmware setup, SO-101 / LeRobot. New during the event: pusher behaviour, dock and bumper design, scan-and-roll station, handoff protocol, DRS client, dashboard.

## Slides (5 only)

1. Title, team, one-line pitch, photo of the bag of bottles.
2. Problem: deposit system since October 2025, 0.50 PLN per container, the bag in the kitchen. **Use only sourced numbers.**
3. Solution diagram (from `02`): CubeBot → dock → SO-101 roll + scan → RoArm → bag/reject → voucher (Kaucja.pl API flow).
4. What we built in 2 days: photos, dashboard screenshot, which Tier works, success rates from the rehearsal log.
5. What's next + credits.

## Q&A prep

| Likely question | Short answer |
|---|---|
| Is it connected to the real deposit system? | The client follows the Kaucja.pl OpenAPI; the API is for registered shops, so the demo uses a mock with the same calls. Switching is a config change once a station ID and credentials are issued |
| How do you know it's a deposit container? | EAN check against a deposit list, same as a shop till: the till accepts or rejects locally, then reports the transaction |
| Why does the second arm roll the bottle? | The barcode can be anywhere on the curved label. Rolling brings it under the camera, like the rollers in a real return machine |
| Why not put an arm on the robot? | 12 small leg servos can't carry a 12 V arm, and fixed arms at a known spot are far more reliable. The dock does the precision |
| Is the pick autonomous? | Triggered, verified and routed automatically; the motion is a taught pose. The dock guarantees the bottle's position |
| How does it find the dock? | An ArUco marker on the back wall, seen by the robot's camera |
| Does it crush the bottles? | No. The gripper angle is taught on a real bottle, because a crushed bottle loses its deposit |
| Why a legged robot for this? | Home floors have rugs, cables and thresholds. And it's the platform we have: we say so openly |
| Hardest problem? | Answer with the real one from the logs |
| Cost? | BOM total from the parts list. Don't guess |

## Submission checklist (R4 owns; done by T-1h)

- [ ] Repo with a clear `README.md`: what it is, how to run (mock and real), diagram, credits, licence.
- [ ] `docs/` in the repo (plan, `arm.md`, `drs-api.md`), plus `decisions.md` with the real gates and fallbacks taken.
- [ ] 60-90 s **video**: one full accept cycle and one reject, counter and voucher visible. Two takes, best one exported.
- [ ] 5 slides exported to PDF.
- [ ] Photos: CubeBot with the bumper, the dock, the SO-101 rolling, the RoArm mid-pick, the dashboard.
- [ ] Dock build notes (dimensions, camera height, lamp position) so anyone can rebuild it.
- [ ] BOM and rough cost.
- [ ] Team members and roles.
- [ ] Rehearsal log saved in `runs/`.
- [ ] Everything on a USB stick and on a laptop that doesn't need internet.

## Rehearsal log template

| # | Time | Pusher mode | Scan (rolls needed) | Decision correct? | Bin correct? | Time per container | Failure cause | Fix |
|---|---|---|---|---|---|---|---|---|
| 1 | | auto | | | | | | |

At least 5 consecutive runs logged before T-1h. Use full `auto` in the demo only if at least 4 of the last 5 passed.
