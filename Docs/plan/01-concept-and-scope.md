# 01: Concept and Scope

## The pitch in three sentences

Every empty bottle or can in a Polish home is worth 0.50 PLN, but only once somebody collects it and returns it. Kaucjobot is a return machine on legs: a walking robot collects empties from the floor, and at its dock two arms scan, sort and bag them. At the end you get a voucher that says exactly how much is in the bag.

Deposit facts for the slide (verify the exact wording before the demo): 0.50 PLN for plastic bottles up to 3 L and metal cans up to 1 L. The container must come back **empty and not crushed**, with the label and barcode readable. That is why our grippers must not squash it and why we scan the EAN.

## Demo arena (built by us, about 2 m x 1.5 m)

```
   +------------------------------------------------------+
   |  [REJECT]  [SO-101]  [ DOCK ]  [RoArm]  [BAG]        |   <- back wall, camera on a stand above the dock
   |                       \    /                         |
   |                        \  /   funnel mouth ~60 cm    |
   |                                                      |
   |        (bottle)           (bottle)     (jar)         |   <- containers lying on the floor
   |                                                      |
   |   [rug patch]                                        |
   |                                                      |
   |             CubeBot START (facing dock)              |
   +------------------------------------------------------+
```

- **Floor:** smooth and flat (bottles roll and slide predictably). Optional rug patch in Tier 2.
- **Containers:** empty 0.5 L PET, label on, cap on, **EANs pre-registered** in `deposit_eans.json`. Plus one non-deposit item for the reject beat (a small glass jar or a carton; check that CubeBot can push it). A strip of coloured tape round the middle of each for the HSV fallback, **not over the barcode**.
- **ArUco marker** on the dock back wall (A5 print), so CubeBot can find the dock.
- **Bag** and **reject bin** in frames, within the RoArm's reach.
- **Overhead webcam + lamp** on a stand, looking straight down at the pocket. Lock focus and exposure.

## The dock: mechanics do the hard part

A **pocket** against the back wall, just longer than a bottle, with a **V-shaped funnel** in front of it:

```
   back wall  ===========================
              |  pocket 24 x 10 cm      |    <- bottle ends here, lying along the wall
              |  floor = V-groove       |       (two strips, so the bottle can roll in place)
               \                       /
                \      funnel         /      <- two angled boards, mouth ~60 cm
                 \                   /
```

- A bottle pushed anywhere into the funnel mouth rolls in, hits the back wall and stops **lying parallel to the wall, in the same place every time**. The pocket is only about 3 cm longer than the bottle, so it can't slide sideways.
- **The pocket floor is a shallow V-groove** (two strips about 4 cm apart). The bottle sits in it like a pencil in a tray. The SO-101 can roll it in place to bring the barcode up, and it doesn't wander off.
- Cap direction (left or right) doesn't matter: the RoArm grips the middle, and the SO-101 rolls from the top.
- Both arms are fixed next to the pocket, so **taught poses** always work.
- Add a small **ramp lip** (1 cm) at the pocket entrance so the bottle doesn't roll back out.
- Materials: cardboard or foamboard, hot glue, tape to the floor. Build a first version in hour 1 and adjust it all day.

## CubeBot bumper

A **V-shaped scoop** on CubeBot's front (cardboard or a 3D print, as light as possible) keeps the bottle centred while the robot walks. Without it a lying bottle rolls off to the side after two steps.

## Autonomy loop

```
     CubeBot                                        Station (laptop + 2 arms + overhead camera)
SEARCH -> APPROACH -> PUSH -> BACK_OFF --"clear"-->  DOCK_CHECK -> SCAN -> DECIDE -> PICK -> BAG or REJECT -> COUNT
  ^                    |                                          |  ^                                      |
  +------ lost / timeout                              SO-101 roll +--+ up to 4 rolls                        |
  +-------------------------------------- "done" <-------------------------------------------------------+
  End of session: "Voucher" button -> POST /transaction (mock DRS) -> voucher QR on screen
  ESTOP reachable from every state (gamepad button + dashboard + keyboard)
```

| State | What happens | Signals |
|---|---|---|
| SEARCH | Turn in place in steps until a container is seen for N frames | Detector |
| APPROACH | Walk toward it, steer to keep it centred, stop when it fills the bumper zone | Detector (box position and size) |
| PUSH | Walk toward the dock marker, keeping it centred. The bumper carries the container | ArUco marker (centre, size) |
| BACK_OFF | Marker large → walk back a fixed number of steps → tell the station "clear" | Marker size, step count |
| DOCK_CHECK | Is something in the pocket? Is the robot out of the arm zone? | Overhead camera ROIs |
| SCAN | Decode the barcode from the overhead camera. If none is found, the SO-101 rolls the bottle a quarter turn and tries again (up to 4 times) | `zxing-cpp` on the frame |
| DECIDE | EAN in `deposit_eans.json` → **accept**; unknown EAN or unreadable after 4 rolls → **reject** | Local list, like a shop till |
| PICK → BAG / REJECT | RoArm: home → above pocket → pick → close → lift → above bag or reject bin → open → home | Taught poses |
| COUNT | Accepted: +1, +0.50 PLN, ding, EAN and product name on screen. Rejected: "not a deposit container" | Dashboard |
| VOUCHER | On button press: `POST /transaction` with all accepted EANs → voucher ID + amount → QR on screen | Mock DRS (`../drs-api.md`) |

**Geometry trick for Tier 1:** CubeBot starts **behind** the containers, facing the dock. So "get behind it, then push toward the dock" is simply "walk to it, then keep walking toward the marker". Tier 2 handles containers that are off that line.

### What is real vs taught vs mocked (say this openly in the pitch)

- **Real, live:** container detection, dock finding, walking and pushing, the handoff between the three robots, barcode reading, accept/reject, counter.
- **Taught:** both arms' poses, recorded once by hand and replayed. The dock is designed so that this is enough.
- **Mocked:** the Kaucja.pl server. Our client uses the endpoints from the official OpenAPI quick start; the server answering today is our mock (unless we get sandbox access).

## Scope tiers

| Tier | Goal | Acceptance test | Ships by |
|---|---|---|---|
| **0: Alive** | CubeBot walks, turns, walks back. Both arms replay poses. Dock built. Overhead camera decodes a barcode. Detection overlay in a browser | Robot walks 1 m and turns 90 deg. RoArm pick+bag of a hand-placed bottle 5 of 5. SO-101 rolls a bottle a quarter turn 5 of 5. EAN decoded from a hand-placed bottle | End of Day 1 |
| **1: MVP (must ship)** | One bottle, placed 80-120 cm in front of the dock on the start line: push → dock → RoArm pick → bag → +0.50 PLN, no hands | 5 trials: at least 3 of 5 end in the bag, each in under 2 min | Day 2, T-6h |
| **2: Target** (in this order) | a) Scan + accept/reject + product name on screen. b) SO-101 roll-to-find-barcode. c) Voucher via the mock DRS (`POST /transaction` + QR). d) 3 containers in a row incl. the reject item. e) One can (standing, own poses). f) Rug crossing | Full run of 3 containers, with the correct bag/reject decision, at least 3 of 5 times | Day 2, T-4h (freeze) |
| **3: Stretch** (only if Tier 2 is boringly reliable) | Bag full → `bag_replacement` with a printed seal code. Real Kaucja.pl sandbox if granted. Voice "Kaucjobot, clean up". Phone push "10 PLN waiting" when internet is available | Any one works twice in a row | Never blocks the demo |

## Explicit cut list (do NOT do)

| Cut | Why |
|---|---|
| Arm vision, hand-eye calibration, grasp planning | The dock puts the bottle in one place, so taught poses are enough |
| Any arm mounted on CubeBot | 12 small leg servos can't carry a 12 V arm. The arms stay at the dock |
| Arm-to-arm handoff in the air | Both arms work on the bottle in the pocket, one after the other. No handoff |
| Deposit-logo recognition by vision | The EAN check is enough and is what real tills do |
| SLAM, maps, path planning | The arena is 2 m; the marker plus the detector is enough |
| Training a custom "can" detector | Hailo compile + dataset takes a day. Cans come in only as Tier 2 with HSV |
| New RL gaits | Use the existing waypoint walk/turn. `crawl.onnx` only if the waypoint gait fails |
| Full containers | Too heavy for the arms, and deposit containers come back empty anyway |
| Fancy UI | One page: video, state, counter in PLN, last EAN, voucher, E-STOP |

## Why this should score

| Typical criterion | How we address it |
|---|---|
| Working demo | Short closed loop, a dock that removes most failure modes, rehearsed 5+ times, backup video |
| Technical depth | Legged locomotion with visual servoing, Hailo on-device detection, marker navigation, a three-robot handoff, barcode scanning with active rolling, integration with a real API spec |
| Usefulness | Real money, a real law, a real bag of bottles in every kitchen, and it rejects non-deposit items like a real machine |
| Wow | Robot pushes a bottle in, arm rolls it, *beep*, the EAN pops up, the arm bags it, counter dings "+0,50 zł", and a voucher comes out at the end |
| Communication | 3-minute script, honest real/taught/mocked framing |
