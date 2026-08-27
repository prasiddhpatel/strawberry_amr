"""
Real, executable unit tests for zeroshot_postprocess.py -- pure
functions, no ROS/PyTorch dependency.

Run: cd src/plant_perception && python3 -m pytest test/test_zeroshot_postprocess.py -v
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'plant_perception'))
import numpy as np
import pytest  # noqa: E402

from zeroshot_postprocess import (
    format_grounding_dino_prompt, match_label_to_phrase,
    backproject_mask_pixels, cluster_centroid_and_plausibility,
)


class TestPromptFormatting:
    def test_single_label_lowercased_and_dotted(self):
        assert format_grounding_dino_prompt(['Ripe Strawberry']) == 'ripe strawberry.'

    def test_already_correct_label_unchanged(self):
        assert format_grounding_dino_prompt(['ripe strawberry.']) == 'ripe strawberry.'

    def test_multiple_labels_joined(self):
        result = format_grounding_dino_prompt(['ripe strawberry', 'weed'])
        assert result == 'ripe strawberry. weed.'

    def test_empty_list_raises(self):
        with pytest.raises(ValueError):
            format_grounding_dino_prompt([])

    def test_whitespace_only_labels_raise(self):
        with pytest.raises(ValueError):
            format_grounding_dino_prompt(['   ', ''])


class TestLabelMatching:
    def test_exact_match(self):
        assert match_label_to_phrase('ripe strawberry',
                                     ['ripe strawberry', 'weed']) == 'ripe strawberry'

    def test_partial_phrase_matches_best_label(self):
        # Grounding DINO sometimes returns a fragment, not the full label
        assert match_label_to_phrase('strawberry',
                                     ['ripe strawberry', 'weed']) == 'ripe strawberry'

    def test_no_overlap_returns_none(self):
        assert match_label_to_phrase('bicycle', ['ripe strawberry', 'weed']) is None

    def test_picks_higher_overlap_not_first(self):
        # 'strawberry plant canopy' shares 2 words with the second label,
        # only 1 with the first -- must not just return the first label
        result = match_label_to_phrase(
            'strawberry plant', ['ripe strawberry', 'strawberry plant canopy'])
        assert result == 'strawberry plant canopy'


class TestBackprojectMaskPixels:
    @staticmethod
    def _reference_loop(xs, ys, depth, W, H, dW, dH, fx, fy, cx, cy,
                        is_mm, dmin, dmax):
        """Independent, deliberately naive re-implementation as a test
        oracle -- if this and the vectorized version disagree, one of
        them is wrong."""
        out = []
        for x, y in zip(xs, ys):
            if not (0 <= x < W and 0 <= y < H):
                continue
            du, dv = int(x * dW / W), int(y * dH / H)
            if not (0 <= du < dW and 0 <= dv < dH):
                continue
            z = float(depth[dv, du])
            z_m = z / 1000.0 if is_mm else z
            if z_m < dmin or z_m > dmax:
                continue
            out.append(((x - cx) * z_m / fx, (y - cy) * z_m / fy, z_m))
        return np.array(out, dtype=np.float64)

    def test_matches_reference_loop_exactly(self):
        rng = np.random.default_rng(1)
        mask = np.zeros((480, 640), dtype=bool)
        mask[150:300, 200:400] = True
        ys, xs = np.nonzero(mask)
        xs, ys = xs[::3], ys[::3]
        depth = rng.integers(300, 2500, size=(480, 640)).astype(np.uint16)

        vec = backproject_mask_pixels(
            xs, ys, depth, (640, 480), (640, 480),
            (500.0, 500.0, 320.0, 240.0), True, 0.15, 3.0)
        ref = self._reference_loop(
            xs, ys, depth, 640, 480, 640, 480,
            500.0, 500.0, 320.0, 240.0, True, 0.15, 3.0)

        assert vec.shape == ref.shape
        assert np.allclose(vec, ref, atol=1e-9)

    def test_metres_depth_not_divided(self):
        depth_m = np.full((480, 640), 0.7, dtype=np.float32)
        pts = backproject_mask_pixels(
            np.array([320]), np.array([240]), depth_m, (640, 480), (640, 480),
            (500.0, 500.0, 320.0, 240.0), False, 0.15, 3.0)
        assert pts.shape[0] == 1
        assert pts[0, 2] == pytest.approx(0.7)

    def test_zero_depth_dropped(self):
        depth = np.zeros((480, 640), dtype=np.uint16)
        pts = backproject_mask_pixels(
            np.array([320]), np.array([240]), depth, (640, 480), (640, 480),
            (500.0, 500.0, 320.0, 240.0), True, 0.15, 3.0)
        assert pts.shape[0] == 0

    def test_empty_input_no_crash(self):
        depth = np.full((480, 640), 800, dtype=np.uint16)
        pts = backproject_mask_pixels(
            np.array([]), np.array([]), depth, (640, 480), (640, 480),
            (500.0, 500.0, 320.0, 240.0), True, 0.15, 3.0)
        assert pts.shape == (0, 3)

    def test_principal_point_projects_to_zero_xy(self):
        depth = np.full((480, 640), 1.2, dtype=np.float32)
        pts = backproject_mask_pixels(
            np.array([320]), np.array([240]), depth, (640, 480), (640, 480),
            (500.0, 500.0, 320.0, 240.0), False, 0.15, 3.0)
        assert pts[0, 0] == pytest.approx(0.0)
        assert pts[0, 1] == pytest.approx(0.0)


class TestClusterPlausibility:
    def test_too_few_points_rejected(self):
        pts = np.random.default_rng(0).normal(size=(5, 3))
        centroid, plausible = cluster_centroid_and_plausibility(pts, min_points=15)
        assert centroid is None
        assert plausible is False

    def test_plant_sized_cluster_plausible(self):
        rng = np.random.default_rng(0)
        # a ~15cm-wide cluster centred at (1.0, 0.2, 0.8) -- plant-scale
        pts = rng.normal(loc=[1.0, 0.2, 0.8], scale=0.03, size=(50, 3))
        centroid, plausible = cluster_centroid_and_plausibility(pts)
        assert plausible is True
        assert centroid == pytest.approx([1.0, 0.2, 0.8], abs=0.05)

    def test_wall_sized_cluster_implausible(self):
        rng = np.random.default_rng(0)
        # points spread across 2m horizontally -- too large to be one plant
        pts = np.column_stack([
            rng.uniform(-1.0, 1.0, 100), rng.uniform(-1.0, 1.0, 100),
            rng.normal(1.5, 0.05, 100)])
        _, plausible = cluster_centroid_and_plausibility(pts, max_extent_m=1.0)
        assert plausible is False

    def test_single_noisy_point_cluster_implausible(self):
        # all points nearly identical -- near-zero extent, likely a single
        # spurious return, not a real plant with actual volume
        pts = np.full((20, 3), 1.0) + np.random.default_rng(0).normal(
            scale=1e-4, size=(20, 3))
        _, plausible = cluster_centroid_and_plausibility(pts, min_extent_m=0.01)
        assert plausible is False

    def test_depth_discontinuity_cluster_implausible(self):
        # A SAM mask bleeding across a depth discontinuity: tight X/Y
        # (looks like a small, plant-sized patch in the image) but two
        # depth bands -- e.g. the near edge of a plant plus the wall/row
        # behind it -- averaging into a centroid that sits in mid-air with
        # nothing actually there. Horizontal extent alone does not catch
        # this; it must be rejected on depth (Z) extent.
        rng = np.random.default_rng(0)
        near = rng.normal(loc=[0.5, 0.5, 0.80], scale=0.01, size=(30, 3))
        far = rng.normal(loc=[0.5, 0.5, 2.30], scale=0.01, size=(30, 3))
        pts = np.vstack([near, far])
        centroid, plausible = cluster_centroid_and_plausibility(pts)
        assert plausible is False
        # sanity check this is really the bimodal-depth scenario, not
        # something else: centroid Z should sit roughly between the two
        # depth bands (~1.55m), not at either real surface
        assert centroid[2] == pytest.approx(1.55, abs=0.05)
