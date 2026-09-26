# 07: Demo

**Done when:** a teammate who didn't build it can run the demo from this page.

## Before the audience arrives (5 minutes)

1. Check the RoArm supply with a multimeter: **7.4–8.4 V**.
2. Power on the Pi, the RoArm and the SO-101. Wait about 20 s for the RoArm to boot.
3. On the Pi:
   ```sh
   cd Raspberry && . .venv/bin/activate
   python roarm_wifi.py where      # real values, not placeholders
   python so101.py go look && python vision.py photo check.jpg
   ```
4. Look at `check.jpg`: if the rim tag has moved more than about 5 px, run `python main.py calibrate` (step 05).
5. Test Brev on the saved photo: `python vision.py ask check.jpg`.

## The run

- Load the bin with the demo set: 2 cans or bottles, 2 paper, 2 plastic, all tested in step 06.
- `python main.py sort`
- Show `last_look.jpg` on a screen: the audience sees what the robot "thinks".

## Reset

Empty the containers and put the demo set back in the bin. Nothing else needs resetting between runs.

## Fallbacks

| Problem | Fallback |
|---|---|
| Brev is down or slow | Run `vision.py ask` on a pre-recorded photo, or swap `ask_llm` for a fixed answer (the `selftest` pattern). |
| The RoArm WiFi IP changed | Find it by MAC (step 01 pitfalls) and update `roarm_ip`. |
| The RoArm shoulder is hot or sagging | Stop. Let it cool. Lower `roarm_spd`, use lighter items. |
| A pick misses | Leave it: the next round takes a new photo and retries. |

## Out of scope for now

`main.py run` (driving the bin along an AprilTag route) needs `set_wheels()` wired to a drive base, which isn't decided yet. Demo with `sort` alone.
