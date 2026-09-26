# 04: Vision (Brev model)

**Done when:** on 10 real `look` photos of mixed trash, the model gets at least 90 % of the bins right, and every grab dot lands on its item.

## Tasks

- [ ] Build a test set: 10–15 photos from the real `look` pose, with real trash in the real bin (`python so101.py go look && python vision.py photo t01.jpg`). Mix the cases:
  - a single item; 3–5 items apart; items touching or overlapping;
  - a standing bottle, a lying bottle, a crushed can;
  - a crumpled paper ball and a flat sheet;
  - a bag lying flat and a bag bunched up;
  - an empty bin (the answer must be `[]`).
- [ ] Run each one: `python vision.py ask t01.jpg`, then look at `answer.jpg` (a red dot and label per item).
- [ ] Keep a short tally (photo, how many right, what went wrong). Change one thing at a time, in this order:
  1. **Prompt** (`PROMPT` in `vision.py`): add examples of items it gets wrong. Tell it to ignore the bin walls, the tape and the AprilTags.
  2. **Lighting:** diffuse and even light, no hard shadows or glare on the plastic. This often fixes more than the prompt does.
  3. **Grab point for tall items:** if standing bottles get dots on their tops, ask for the point where the item touches the floor rather than its centre. That also fixes most of the parallax error in step 06.
  4. **Model:** only if 1–3 aren't enough. Try a bigger Qwen2.5-VL on Brev; `brev_model` is just a setting.
- [ ] Measure latency. A reply should take a few seconds; if it's over 10 s, shrink `LLM_SIZE` (keep it a multiple of 28).

## Pitfalls

- The model answers in `LLM_SIZE` pixels. `ask_llm` scales the points back to the camera's pixels, so leave that scaling alone.
- `parse_items` drops entries with an unknown bin or out-of-frame points. Items that go missing from the list are often a label typo in the model's answer, so check the raw reply.
