# Jetson AGX Orin 64GB — OS and Software Setup

Getting the AGX Orin to the exact software state this workspace needs.

## Part 1 — Which OS, exactly

**You need JetPack 6.x, which is Ubuntu 22.04 (L4T R36.x).**

This is not a preference — it's a hard requirement:

| JetPack | Ubuntu | ROS 2 Humble via apt? |
|---|---|---|
| JetPack 5.x | 20.04 | **No.** Humble has no 20.04 packages. |
| **JetPack 6.x** | **22.04** | **Yes** — this is what you need. |

ROS 2 Humble's apt packages exist only for Ubuntu 22.04, so JetPack 5.x
(Ubuntu 20.04) leaves you building Humble from source or running it in a
container. JetPack 6 removes that problem entirely. NVIDIA's own Isaac
ROS 3.0 is built on Humble and tested on the AGX Orin, which is the same
alignment.

### Check what you currently have

```bash
cat /etc/nv_tegra_release        # L4T version; R36.x == JetPack 6
lsb_release -a                   # should say Ubuntu 22.04
dpkg -l | grep nvidia-jetpack    # JetPack meta-package version
uname -m                         # aarch64 (expected -- this is an ARM64 board)
```

If `lsb_release` says 22.04 and `nv_tegra_release` says R36, you're
already correct and can go straight to Part 3.

## Part 2 — If you're on JetPack 5.x / Ubuntu 20.04

### Do NOT run `do-release-upgrade`

**This is the single most important warning in this guide.** Upgrading
20.04 → 22.04 in place on a Jetson breaks the JetPack/NVIDIA driver
stack. The Ubuntu userspace moves but the L4T BSP, CUDA, and kernel
modules do not — you end up with an Ubuntu 22.04 that has no working
GPU stack, and the documented recovery is to flash properly anyway. This
is a real, reported failure mode, not a theoretical one.

### Flash JetPack 6 properly instead

Flashing is done **from a separate x86_64 host PC running Ubuntu**, not
from the Orin itself:

1. On the host PC, install **NVIDIA SDK Manager**
   (`developer.nvidia.com/sdk-manager`). An NVIDIA developer account is
   required (free).
2. Connect the Orin to the host with the **USB-C cable** (the port next
   to the 40-pin header, not the power port).
3. Put the Orin into **Force Recovery Mode**: with the board powered off,
   hold the middle (Force Recovery) button, press and release Reset while
   still holding Recovery, then release Recovery. Confirm on the host
   with `lsusb | grep -i nvidia`.
4. In SDK Manager: select **Jetson AGX Orin**, target **JetPack 6.x**,
   and include the **Jetson Linux** and **Jetson SDK Components**
   (CUDA, cuDNN, TensorRT — you need all three for the YOLO path).
5. Flash, then complete the Ubuntu first-boot setup on the Orin.

**Budget real time for this** — the download alone is tens of GB, and the
whole process typically runs over an hour. Do it before you need the
robot working, not the night before.

## Part 3 — ROS 2 Humble

Standard apt install; nothing Jetson-specific. ROS 2 Humble publishes
`arm64` packages for Ubuntu 22.04, so this Just Works on the Orin:

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && source ~/.bashrc
sudo rosdep init && rosdep update
```

`$(dpkg --print-architecture)` resolves to `arm64` here — that's correct,
don't hardcode `amd64`.

Then this workspace's own dependencies:
```bash
cd ~/strawberry_ws && ./scripts/install_deps.sh
```

## Part 4 — Verify the GPU stack

Before trusting anything GPU-related:

```bash
nvcc --version                    # CUDA compiler
dpkg -l | grep -i tensorrt        # TensorRT libraries
sudo /usr/bin/jetson_clocks --show   # clocks/power state
```

**`nvidia-smi` does not work on Jetson** and its absence is not a fault —
Jetson has an integrated GPU sharing memory with the CPU, not a discrete
PCIe card. Use `tegrastats` instead:

```bash
sudo tegrastats     # live CPU/GPU/RAM/thermal; Ctrl-C to stop
```

**Set the power mode to maximum** before any real run — the Orin ships in
a lower-power mode by default, and leaving it there is a common cause of
"why is inference slow":
```bash
sudo nvpmodel -q                  # query current mode
sudo nvpmodel -m 0                # MAXN, maximum performance
sudo jetson_clocks                # pin clocks to max
```
`nvpmodel -m 0` persists across reboots; `jetson_clocks` does not — re-run
it after each boot, or add it to a startup script.

## Part 5 — ONNX Runtime for the YOLO detector (only if using `detector:='yolo'`)

**Do not `pip install onnxruntime`** — the PyPI wheel is CPU-only, and
the node will fail loudly at startup telling you so. **Do not
`pip install onnxruntime-rocm`** either — that's AMD's stack, x86_64
only, and will not install on this ARM64 board at all. It was correct for
the previous laptop-based architecture and is simply wrong here.

You need **NVIDIA's own Jetson build of `onnxruntime-gpu`**, which
exposes `TensorrtExecutionProvider` and `CUDAExecutionProvider`. NVIDIA
publishes Jetson wheels at `elinux.org/Jetson_Zoo` (ONNX Runtime
section) — match the wheel to your **JetPack version and Python
version**, since these are built per-JetPack, not universal.

Verify it actually took:
```bash
python3 -c "import onnxruntime as ort; print(ort.get_available_providers())"
```
You must see `TensorrtExecutionProvider` and/or `CUDAExecutionProvider`
in that list. If you only see `['CPUExecutionProvider', 'AzureExecutionProvider']`,
you have the wrong wheel — this specific symptom is common and is a wheel
problem, not a configuration problem.

**First TensorRT run is slow, once.** TensorRT compiles an optimized
engine for your exact model and this exact GPU, which can take several
minutes. `plant_detector_yolo_params.yaml` sets `trt_cache_dir` so that
cost is paid once rather than on every node start — if you see multi-
minute startups repeatedly, that cache path isn't being written.

## Part 6 — Networking

The AGX Orin Developer Kit **ships with WiFi already installed** — an
AzureWave AW-CB375NF 802.11a/b/g/n/ac + Bluetooth 5.0 module in the M.2
Key E slot, dual-band 2.4/5 GHz. No extra hardware needed to join the
Pi's access point.

JetPack uses NetworkManager, so `scripts/setup_network_pi_orin.sh orin`
works as written (it uses `nmcli`). Run the Pi role first — it hosts the
AP and must be broadcasting before the Orin can join:

```bash
./scripts/setup_network_pi_orin.sh pi      # on the Raspberry Pi
./scripts/setup_network_pi_orin.sh orin    # on the AGX Orin
```

The Orin also has 10GbE, but the robot moves and the Orin doesn't, so
WiFi is the link that matters. You can still use Ethernet for the Orin's
own internet access — a separate interface, no conflict with the AP link.

## Part 7 — Storage

The devkit has 64GB eMMC, which is tight once JetPack, ROS 2, and a
workspace are installed. The M.2 Key M slot takes an NVMe SSD, and moving
the root filesystem to NVMe is the standard fix if you run short. Check
with `df -h /` before assuming you have room for maps and rosbags.

## Known-good verification sequence

Run these in order; each one gates the next:

```bash
lsb_release -a                                   # Ubuntu 22.04
cat /etc/nv_tegra_release                        # R36.x
ros2 doctor --report | head -20                  # ROS 2 sane
nvcc --version                                   # CUDA present
sudo tegrastats --interval 1000                  # GPU/thermal alive (Ctrl-C)
ping 192.168.30.1                                # Pi reachable over the AP
ros2 topic list                                  # Pi's topics visible here
```

If all seven pass, the Orin is ready for
`ALL_IN_ONE_DEPLOYMENT_GUIDE.md` Part 5 onward.
