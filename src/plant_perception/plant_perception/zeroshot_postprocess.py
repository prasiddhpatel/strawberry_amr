"""
Pure, ROS/PyTorch-independent post-processing for the zero-shot
(Grounding DINO + MobileSAM) plant detector -- deliberately separated
from the actual inference node (plant_detector_zeroshot_node.py), same
reasoning as command_safety.py, reward.py, and yolo_postprocess.py
elsewhere in this workspace.

This is the part most worth getting right through review and real tests.
It is also, specifically, a second attempt at code this workspace has
gotten wrong before: the removed YOLO node's inline per-pixel
back-projection loop measured at 101 ms/detection (152% of the 15 FPS
frame budget) before being rewritten as the vectorized version below.
That lesson is applied here from the start rather than discovered later.
"""
import numpy as np


def format_grounding_dino_prompt(labels):
    """
    Grounding DINO's HuggingFace `transformers` integration requires
    lowercase text queries, each ending with a period -- confirmed
    directly against IDEA-Research's own model card examples, not
    assumed. Getting this wrong doesn't raise an error; it silently
    produces poor-quality or zero detections, which is a much worse
    failure mode than a crash.

    labels: list of str, e.g. ['ripe strawberry', 'strawberry plant canopy']
    Returns: a single prompt string, e.g. 'ripe strawberry. strawberry plant canopy.'
    """
    if not labels:
        raise ValueError("format_grounding_dino_prompt: labels must be non-empty")
    parts = []
    for label in labels:
        clean = label.strip().lower()
        if not clean:
            continue
        if not clean.endswith('.'):
            clean = clean + '.'
        parts.append(clean)
    if not parts:
        raise ValueError("format_grounding_dino_prompt: no non-empty labels")
    return ' '.join(parts)


def match_label_to_phrase(detected_phrase, candidate_labels):
    """
    Grounding DINO returns a free-text phrase per detection (its own
    interpretation of which words in the prompt it matched), not a clean
    class index -- matching it back to one of our configured labels needs
    a real (if simple) comparison, not an exact-string assumption, since
    the returned phrase can be a substring or slightly reworded fragment
    of the original label.

    Returns the best-matching label from candidate_labels, or None if
    nothing matches at all (kept rather than silently defaulting to the
    first label, which would mislabel a genuine detection).
    """
    phrase = detected_phrase.strip().lower()
    if not phrase:
        return None
    best, best_score = None, 0
    for label in candidate_labels:
        clean_label = label.strip().lower()
        # score = count of label words present in the returned phrase
        label_words = set(clean_label.split())
        phrase_words = set(phrase.split())
        score = len(label_words & phrase_words)
        if score > best_score:
            best, best_score = label, score
    return best


def backproject_mask_pixels(xs, ys, depth, color_wh, depth_wh, intrinsics,
                            depth_is_mm, depth_min_m, depth_max_m):
    """
    Back-project mask pixels (in COLOUR-image pixel space) to 3D points
    in the camera optical frame. Fully vectorized -- see this module's
    own docstring for why that matters, and
    yolo_postprocess.backproject_masked_pixels (removed with the YOLO
    node, this is its direct successor) for the original benchmark.

    xs, ys      : integer pixel coords in COLOUR-image space (Grounding
                  DINO/MobileSAM operate on the colour frame directly --
                  no letterbox/model-input-space conversion needed here,
                  unlike the removed YOLO node, which is one real
                  simplification this design gets over that one)
    depth       : (dH, dW) raw depth array, uint16 (mm) or float32 (m)
    color_wh    : (W, H) of the colour image the mask was computed on
    depth_wh    : (dW, dH) of the depth image (may differ from colour)
    intrinsics  : (fx, fy, cx, cy) of the COLOUR camera
    Returns: (N,3) float64 array of (X, Y, Z) in metres.
    """
    W, H = color_wh
    dW, dH = depth_wh
    fx, fy, cx, cy = intrinsics

    xs = np.asarray(xs, dtype=np.float64)
    ys = np.asarray(ys, dtype=np.float64)
    if xs.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    keep = (xs >= 0) & (xs < W) & (ys >= 0) & (ys < H)
    xs, ys = xs[keep], ys[keep]
    if xs.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    du = (xs * dW / W).astype(np.int32)
    dv = (ys * dH / H).astype(np.int32)
    keep = (du >= 0) & (du < dW) & (dv >= 0) & (dv < dH)
    xs, ys, du, dv = xs[keep], ys[keep], du[keep], dv[keep]
    if xs.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    z = depth[dv, du].astype(np.float64)
    if depth_is_mm:
        z = z / 1000.0
    keep = (z >= depth_min_m) & (z <= depth_max_m)   # also drops z<=0
    xs, ys, z = xs[keep], ys[keep], z[keep]
    if xs.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    X = (xs - cx) * z / fx
    Y = (ys - cy) * z / fy
    return np.stack([X, Y, z], axis=1)


def cluster_centroid_and_plausibility(points_3d, min_points=15,
                                      max_extent_m=1.0, min_extent_m=0.01):
    """
    A deliberately lightweight geometric sanity check -- NOT a
    replacement for the removed plant_geometry_filter's PCL plane-removal
    and outlier filtering, which needed an uncompiled C++ package and a
    library never verified to build in this project. This is a simpler,
    dependency-free plausibility gate that runs in pure numpy: does this
    cluster of back-projected points have a spatial extent consistent
    with a real plant, or does it look like a flat wall/floor slice or a
    single noisy pixel?

    Open-set models are more prone to false positives than a
    closed-set detector (stated in docs/ZERO_SHOT_PERCEPTION_GUIDE.md);
    this is the first, cheap line of defence before a detection is
    trusted enough to publish.

    Returns (centroid: (3,) array or None, plausible: bool).
    centroid is None if there are too few points to compute one
    meaningfully.
    """
    points_3d = np.asarray(points_3d, dtype=np.float64)
    if points_3d.shape[0] < min_points:
        return None, False

    centroid = points_3d.mean(axis=0)
    extent = points_3d.max(axis=0) - points_3d.min(axis=0)
    horizontal_extent = float(np.linalg.norm(extent[:2]))  # X-Y spread
    depth_extent = float(extent[2])                        # Z spread

    # too small (horizontal): likely a handful of noisy/spurious points,
    #   not a plant
    # too large (horizontal): likely a wall, floor, or a mask that bled
    #   across objects
    # too large (depth): a SAM mask that bleeds across a depth
    #   discontinuity -- a well-known promptable-segmentation failure mode
    #   at object edges -- produces a centroid averaging two unrelated
    #   depths (e.g. a plant's near edge and a wall behind it), which tight
    #   X/Y extent alone does not catch. Only an upper bound is applied on
    #   depth, not a lower one: a real plant viewed close to square-on can
    #   legitimately have very little depth variation, which is not itself
    #   evidence of a spurious cluster the way near-zero horizontal extent is.
    plausible = (min_extent_m <= horizontal_extent <= max_extent_m
                and depth_extent <= max_extent_m)
    return centroid, plausible
