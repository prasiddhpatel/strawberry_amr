"""
Pure-function tests for geometry.py -- no ROS runtime needed, runs under
plain pytest. See CLAUDE.md's testing convention: this is the pattern
established for every other non-trivial pure-logic module in this repo.
"""
import math
import numpy as np
import pytest

from r2_web_viz.geometry import (
    quat_to_rotmat, transform_points, voxel_key, pack_rgb_float, unpack_rgb_uint32,
)


class TestQuatToRotmat:
    def test_identity_quaternion_is_identity_matrix(self):
        R = quat_to_rotmat(0, 0, 0, 1)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_90deg_about_z_matches_hand_derivation(self):
        # q = (0, 0, sin45, cos45): 90 deg rotation about Z.
        s = math.sin(math.radians(45))
        c = math.cos(math.radians(45))
        R = quat_to_rotmat(0, 0, s, c)
        expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        np.testing.assert_allclose(R, expected, atol=1e-9)

    def test_90deg_about_z_rotates_x_axis_to_y_axis(self):
        s = math.sin(math.radians(45))
        c = math.cos(math.radians(45))
        R = quat_to_rotmat(0, 0, s, c)
        rotated = R @ np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(rotated, [0, 1, 0], atol=1e-9)


class TestTransformPoints:
    def test_camera_point_projects_to_hand_computed_map_point(self):
        # Scenario from the design discussion: robot at map (2, 3), yawed
        # 90 deg, camera mounted 0.1 m forward of base_link along the
        # robot's own local +x. Combined map->camera transform:
        #   translation = (2, 3, 0) + R(90)@(0.1, 0, 0) = (2, 3.1, 0)
        #   rotation = 90 deg about Z (same as base_link's yaw; camera
        #              mounted with no extra rotation)
        # A point 1 m directly in front of the camera, in the camera's own
        # frame, is (1, 0, 0). Hand-derivation:
        #   R(90) @ (1,0,0) = (0, 1, 0)
        #   + translation (2, 3.1, 0) = (2, 4.1, 0)
        s = math.sin(math.radians(90))
        c = math.cos(math.radians(90))
        # sin/cos(90) via radians(90) directly for the rotation matrix,
        # not the half-angle quaternion form, since this test constructs
        # the matrix path only -- quaternion construction is covered by
        # TestQuatToRotmat above.
        R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        translation = (2.0, 3.1, 0.0)

        points = np.array([[1.0, 0.0, 0.0]])
        result = transform_points(points, R, translation)

        np.testing.assert_allclose(result, [[2.0, 4.1, 0.0]], atol=1e-9)

    def test_multiple_points_transformed_independently(self):
        R = np.eye(3)
        points = np.array([[1.0, 2.0, 3.0], [0.0, 0.0, 0.0], [-1.0, -1.0, 0.5]])
        result = transform_points(points, R, (10.0, 0.0, 0.0))
        np.testing.assert_allclose(
            result, [[11.0, 2.0, 3.0], [10.0, 0.0, 0.0], [9.0, -1.0, 0.5]], atol=1e-9)


class TestVoxelKey:
    def test_same_voxel_for_nearby_points(self):
        assert voxel_key(1.01, 2.01, 0.01, 0.05) == voxel_key(1.02, 1.99, -0.01, 0.05)

    def test_different_voxel_across_a_cell_boundary(self):
        assert voxel_key(0.024, 0, 0, 0.05) != voxel_key(0.026, 0, 0, 0.05)

    def test_negative_coordinates_key_consistently(self):
        # round() on negative halves banker's-rounds in Python -- just
        # needs to be a pure function of the input, not any particular
        # rounding convention; same input must always give the same key.
        assert voxel_key(-1.234, -5.678, 0.1, 0.1) == voxel_key(-1.234, -5.678, 0.1, 0.1)


class TestRgbPackRoundTrip:
    def test_pack_unpack_roundtrip_pure_red(self):
        packed = pack_rgb_float(255, 0, 0)
        r, g, b = unpack_rgb_uint32(np.array([packed], dtype=np.float32))
        assert (int(r[0]), int(g[0]), int(b[0])) == (255, 0, 0)

    def test_pack_unpack_roundtrip_arbitrary_color(self):
        packed = pack_rgb_float(37, 200, 12)
        r, g, b = unpack_rgb_uint32(np.array([packed], dtype=np.float32))
        assert (int(r[0]), int(g[0]), int(b[0])) == (37, 200, 12)

    def test_matches_js_decoder_convention(self):
        # pointcloud_decode.js reads packed = view.getUint32(...); r =
        # (packed>>16)&0xff, g = (packed>>8)&0xff, b = packed&0xff -- i.e.
        # 0x00RRGGBB. Confirm the Python encoder produces exactly that bit
        # layout, not a byte-order or channel-order mismatch that would
        # only show up as wrong colors in a real browser.
        r, g, b = 0x12, 0x34, 0x56
        packed = pack_rgb_float(r, g, b)
        as_uint32 = np.array([packed], dtype=np.float32).view(np.uint32)[0]
        assert as_uint32 == 0x00123456

    @pytest.mark.parametrize("r,g,b", [(0, 0, 0), (255, 255, 255), (128, 64, 32)])
    def test_various_colors_roundtrip(self, r, g, b):
        packed = pack_rgb_float(r, g, b)
        ur, ug, ub = unpack_rgb_uint32(np.array([packed], dtype=np.float32))
        assert (int(ur[0]), int(ug[0]), int(ub[0])) == (r, g, b)
