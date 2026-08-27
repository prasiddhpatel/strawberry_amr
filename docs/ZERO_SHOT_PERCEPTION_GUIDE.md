# Zero-Shot Perception Guide — Grounding DINO + MobileSAM

Open-vocabulary plant/weed detection. No training, no dataset, no
labelling. **Status: implemented** —
`plant_detector_zeroshot_node.py` replaces the removed YOLO path
entirely. `detector:='hsv'` (default) remains the always-on, GPU-free
baseline; `detector:='zeroshot'` adds plant-versus-weed discrimination,
triggered on demand, not continuously.

---

## Part 0 — Two things to understand before anything else

**You never type prompts at runtime.** `"ripe strawberry"`, `"weed"` are
fixed strings in `plant_detector_zeroshot_params.yaml`, set once. The
node runs automatically from there. There is no interactive/chat mode.

**This does not run every frame.** Grounding DINO + MobileSAM chained
together take hundreds of milliseconds; your camera budget at 15 FPS is
66.7 ms. The node sits idle, caching the latest synced RGB-D frame, until
it receives a message on `/zeroshot_detect_trigger` — then it runs
**one** detection pass and publishes the result. `mission_control` is
expected to publish that trigger when the robot has paused at a plant.
You can also trigger it by hand for testing (Part 6).

---

## Part 1 — Cost and where things live (your original questions)

**Free.** Grounding DINO and MobileSAM are both **Apache 2.0** — verified
against their actual licence files, not assumed. No account, no fee, no
commercial restriction.

**Everything runs on the Orin. Nothing on the Pi.** The Pi's job is
unchanged: sensors, compressed frames, drive commands. Grounding DINO
(`IDEA-Research/grounding-dino-tiny`, ~172M params) needs a few GB VRAM;
MobileSAM's encoder is 5.78M parameters (versus 632M in full SAM). Your
Orin has 64 GB unified memory — not a constraint.

---

## Part 2 — What changed from the original plan, and why

**Grounding DINO now loads via HuggingFace `transformers`**
(`AutoModelForZeroShotObjectDetection` + `AutoProcessor`,
`IDEA-Research/grounding-dino-tiny`), not the `IDEA-Research/GroundingDINO`
GitHub repository this guide originally pointed at. Verified reason: that
repository's own README states it compiles a custom CUDA extension at
install time and falls back to CPU-only if that compile fails — a real,
documented source of breakage on Jetson specifically. The `transformers`
integration is pure PyTorch. No compile step, no extension.

**MobileSAM is unchanged** — its own package
(`pip install git+https://github.com/ChaoningZhang/MobileSAM.git`, confirmed:
it is genuinely not on PyPI) has never needed a compiled extension either
way, so there was nothing to fix there.

**A lightweight geometric plausibility check replaced the full PCL
RANSAC filter.** An earlier revision of this workspace had a
`plant_geometry_filter` C++/PCL package for exactly this purpose; it was
removed along with YOLO and never compiled or verified. Rather than
reintroduce an unverified C++ dependency the night before hardware
bring-up, `plant_detector_zeroshot_node.py` does a dependency-free
plausibility check in pure numpy — rejects detections whose 3D point
cluster is too small (noise) or too large in either horizontal extent
(a wall/floor slice, not one plant) or depth extent (a foreground/
background pair that shares a horizontal footprint in the image but
sits at two genuinely different distances from the camera — a gap the
horizontal-only check originally missed, closed by adding a matching
upper-bound check along the camera's depth axis). Simpler, not
equivalent — open-set false positives are still a real risk this
doesn't fully solve, stated honestly.

---

## Part 3 — Why zero-shot instead of YOLO (unchanged reasoning)

**Gain:** no dataset/labelling/training — that was the longest lead-time
item removed from the schedule. Plant-versus-weed discrimination, which
HSV structurally cannot do (changing a prompt is a config edit, not a
retraining cycle).

**Cost, stated honestly:** slower per-inference (hundreds of ms, hence
event-gating, not a continuous 15 FPS path). More prone to open-set
errors (confident detections of the wrong thing) than a closed-set
model — mitigated, not eliminated, by the plausibility gate above.

---

## Part 4 — Setup on the Orin

**Prerequisite: JetPack 6.x, CUDA confirmed working**
(`docs/JETPACK_SETUP_GUIDE.md`). Verify before anything below:
```bash
nvcc --version
```

### 4.1 PyTorch — NVIDIA's Jetson build, not `pip install torch`

The PyPI `torch` wheel is x86_64/CPU-only on this board. You need
NVIDIA's own Jetson-built wheel, matching your exact JetPack/CUDA
version. Check NVIDIA's Jetson Zoo / forum post for the current
download for your JetPack version, then:
```bash
pip3 install --break-system-packages /path/to/torch-<version>-cp310-cp310-linux_aarch64.whl
```
Verify immediately, before installing anything else:
```bash
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```
`torch.cuda.is_available()` must print `True`. If it doesn't, stop —
nothing past this point will work, and the zero-shot node's own
`force_cuda` check will (correctly) refuse to start until this is fixed.

### 4.2 The rest

```bash
cd ~/strawberry_ws/src/plant_perception
pip install -r requirements_zeroshot.txt --break-system-packages
pip install git+https://github.com/ChaoningZhang/MobileSAM.git --break-system-packages
```

### 4.3 Weights

**Grounding DINO downloads automatically** the first time the node runs
— `transformers` caches `IDEA-Research/grounding-dino-tiny` to
`~/.cache/huggingface/` on first load. No manual download step, but the
first launch will pause for it — expect this, don't assume it hung.

**MobileSAM's checkpoint needs a manual download** (~40 MB):
```bash
mkdir -p ~/strawberry_ws/models
wget https://raw.githubusercontent.com/ChaoningZhang/MobileSAM/master/weights/mobile_sam.pt \
    -O ~/strawberry_ws/models/mobile_sam.pt
```
Point `plant_detector_zeroshot_params.yaml`'s `mobile_sam_checkpoint` at
this path — the node raises a clear error at startup if it's missing or
empty, rather than failing confusingly mid-inference.

### 4.4 Verify standalone, before ROS

Debugging a model problem and a ROS problem at the same time is the most
common way to lose a day. Test the models on a saved image first:
```python
import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

device = "cuda"
processor = AutoProcessor.from_pretrained("IDEA-Research/grounding-dino-tiny")
model = AutoModelForZeroShotObjectDetection.from_pretrained(
    "IDEA-Research/grounding-dino-tiny").to(device)

image = Image.open("test_plant.jpg").convert("RGB")
# lowercase, period-terminated -- required by the API, not a style choice
text = "ripe strawberry. weed."
inputs = processor(images=image, text=text, return_tensors="pt").to(device)
with torch.no_grad():
    outputs = model(**inputs)
results = processor.post_process_grounded_object_detection(
    outputs, inputs.input_ids, box_threshold=0.35, text_threshold=0.25,
    target_sizes=[image.size[::-1]])[0]
print(results["labels"], results["scores"])
```
**Time it.** That number tells you how far apart your trigger events need
to be.

---

## Part 5 — Prompt tuning (this is your equivalent of HSV's six numbers)

| Prompt | Behaviour |
|---|---|
| `"strawberry"` | Broad — may fire on packaging, other red objects |
| `"ripe strawberry"` | Narrower |
| `"strawberry plant canopy"` | The plant, not the fruit |
| `"weed"` | Your least reliable prompt — very open-set |

Two thresholds (`box_threshold`, `text_threshold`, default 0.35/0.25):
**start permissive, then tighten.** Too tight looks identical to "the
model is broken" and sends you debugging the wrong thing. Tune on real
Lab 3003 images, same discipline as HSV.

---

## Part 6 — Running it

```bash
ros2 launch robot_bringup pi_missionbrain_phase1_mapping.launch.py detector:=zeroshot
```

**During an actual autonomous mission** (`phase:='nav'` or `'explore'`),
this is all you need — `mission_control_node` automatically publishes to
`/zeroshot_detect_trigger` the instant Nav2 confirms it has reached each
approached plant, with no further action from you. `detector:=zeroshot`
alone is sufficient; it also sets `mission_control`'s `zeroshot_enabled`
parameter from the same argument, so the two stay in sync.

**For testing in isolation**, before or without a real mission running,
trigger a detection pass by hand against whatever frame the camera
currently sees:
```bash
ros2 topic pub --once /zeroshot_detect_trigger std_msgs/msg/Empty
```
Watch the result:
```bash
ros2 topic echo /plant_detector_zeroshot/status
ros2 topic echo /plant_targets
```

**Confirm CUDA is genuinely active** — the node's startup log line
(`Zero-shot models loaded on device: cuda`) confirms this directly; if it
says `cpu`, `force_cuda` should have already stopped the node from
starting at all, so seeing `cpu` here means `force_cuda` was manually set
to `false`.

---

## Part 7 — Troubleshooting

| Symptom | Likely cause |
|---|---|
| Node refuses to start, `force_cuda` error | Wrong torch wheel (Part 4.1) — check `torch.cuda.is_available()` directly first |
| Node refuses to start, `mobile_sam_checkpoint` error | Path empty or wrong in the yaml, or the file wasn't actually downloaded |
| First launch pauses for a long time | Normal — Grounding DINO's first-run HuggingFace download. Not a hang. |
| Detects nothing | Thresholds too high — drop both to ~0.2 and confirm anything fires at all |
| Detects confidently but wrongly | Open-set error. Tighten the prompt; the plausibility gate helps but doesn't eliminate this |
| Trigger published but nothing happens | Check `/camera/color/image_raw_decompressed` and depth are actually publishing — the node needs a synced frame cached before a trigger can act on it |
| Inference far slower than the standalone test in 4.4 | Check Orin power mode: `sudo nvpmodel -m 0` and `sudo jetson_clocks` |

---

## Part 8 — Honest expectations

- **Day 1:** PyTorch verification, package install, weights, the
  standalone test in Part 4.4.
- **Day 2:** prompt tuning on real Lab 3003 images, trigger timing.
- **Day 3:** ~~wiring the trigger to an actual "arrived at plant" mission
  event~~ — **already done.** `mission_control_node.py` fires
  `/zeroshot_detect_trigger` automatically the instant Nav2 confirms it
  has reached an approached plant, fire-and-forget (see that node's own
  "ZERO-SHOT INTEGRATION" docstring section). Set `detector:=zeroshot` on
  the launch line and this happens with no further wiring — the manual
  `ros2 topic pub` in Part 6 above is for testing the detector in
  isolation, not something you need in normal autonomous operation. What
  Day 3 actually still means: confirm this fires correctly against a
  real approach, on real hardware, not just read the code and assume it
  does.

**HSV remains what the core thesis deliverable depends on.** This is a
genuine enhancement on top of a system that already works without it —
build and prove HSV-based autonomy first.
