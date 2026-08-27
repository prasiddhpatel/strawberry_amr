"""
Real, executable unit tests for generate_polytunnel_world.py -- fully pure,
stdlib-only, no rclpy/numpy dependency. Unlike most other node files in
this workspace, this script needed zero extraction to become testable: it
imports nothing beyond argparse at module scope.

Run: cd src/strawberry_amr_gazebo && python3 -m pytest test/ -v
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import pytest  # noqa: E402

from generate_polytunnel_world import (  # noqa: E402
    build_world, post_model, trough_model, plant_model, threshold_model, main,
)


class TestPostModel:
    def test_pose_and_geometry_reflect_inputs(self):
        xml = post_model('post0', x=1.5, y=-0.3, height=1.1, radius=0.02)
        assert 'name="post0"' in xml
        assert '<pose>1.500 -0.300 0.550 0 0 0</pose>' in xml  # height/2
        assert '<radius>0.02</radius>' in xml
        assert '<length>1.100</length>' in xml


class TestTroughModel:
    def test_length_and_centre_computed_from_span(self):
        xml = trough_model('trough0', x0=0.0, x1=8.0, y=0.2, height=1.1, width=0.3)
        assert '<pose>4.000 0.200 1.100 0 0 0</pose>' in xml  # cx = midpoint
        assert '<size>8.000 0.300 0.15</size>' in xml  # length = x1 - x0


class TestPlantModel:
    def test_foliage_and_two_berries_present_with_correct_z_offset(self):
        xml = plant_model('plant0', x=1.0, y=0.2, trough_height=1.1)
        assert 'name="plant0_foliage"' in xml
        assert '<pose>1.000 0.200 1.180 0 0 0</pose>' in xml  # trough_height + 0.08
        assert xml.count('name="plant0_berry_') == 2


class TestThresholdModel:
    def test_width_and_centre_from_y_span(self):
        xml = threshold_model('thr0', x=2.0, y0=-0.2, y1=0.2, height=0.012)
        assert '<size>0.030 0.400 0.0120</size>' in xml  # width = y1 - y0
        assert '<pose>2.000 0.000 0.0060 0 0 0</pose>' in xml  # cy midpoint, height/2


class TestBuildWorldGeometry:
    def test_posts_and_trough_generated_for_known_row_length_and_spacing(self):
        sdf = build_world(
            num_rows=1, row_spacing=1.0, row_length=6.0, post_spacing=2.0,
            plant_spacing=10.0,  # deliberately sparse -- keep this test focused on posts
            trough_height=1.1, trough_width=0.3, post_radius=0.02, half_width=0.3,
        )
        # px = 0, 2, 4, 6 -- 6 is not > row_length(6.0), so all 4 survive the guard
        assert sdf.count('name="row0_L_post') == 4
        assert sdf.count('name="row0_R_post') == 4
        assert 'name="row0_L_trough"' in sdf
        assert 'name="row0_R_trough"' in sdf

    def test_plant_count_for_known_row_length_and_spacing(self):
        sdf = build_world(
            num_rows=1, row_spacing=1.0, row_length=6.0, post_spacing=100.0,
            plant_spacing=1.0, trough_height=1.1, trough_width=0.3,
            post_radius=0.02, half_width=0.3,
        )
        # plx = 0.15, 1.15, ..., 5.15 -- all <= row_length-0.05 (5.95) -- 6 per side
        assert sdf.count('_foliage"') == 12  # 6 plants x 2 sides

    def test_short_row_still_guarantees_at_least_two_posts_per_side(self):
        """Regression test: max(2, int(row_length/post_spacing)+1) signals
        'always at least 2 posts' (RANSAC needs >=2 points for a line fit),
        but the boundary filter (px > row_length) could silently drop back
        below that floor for a short row combined with wide post_spacing --
        row_length=1.0, post_spacing=2.0 requested 2 posts but only the
        x=0 one used to survive the filter."""
        sdf = build_world(
            num_rows=1, row_spacing=1.0, row_length=1.0, post_spacing=2.0,
            plant_spacing=100.0, trough_height=1.1, trough_width=0.3,
            post_radius=0.02, half_width=0.3,
        )
        assert sdf.count('name="row0_L_post') == 2
        assert sdf.count('name="row0_R_post') == 2

    def test_plant_count_accounts_for_placement_margin_not_just_row_length(self):
        """Regression test: the previous int(row_length / plant_spacing)
        formula ignored the 0.20 m total placement margin (0.15 m start +
        0.05 m end) the boundary check actually enforces, under-generating
        by one plant whenever plant_spacing doesn't divide evenly into the
        margin-adjusted span. row_length=8.0, plant_spacing=0.9: the old
        formula requested int(8.0/0.9)=8, but 9 positions (pl=0..8, plx up
        to 7.35) actually fit within [0.15, row_length-0.05]."""
        sdf = build_world(
            num_rows=1, row_spacing=1.0, row_length=8.0, post_spacing=100.0,
            plant_spacing=0.9, trough_height=1.1, trough_width=0.3,
            post_radius=0.02, half_width=0.3,
        )
        assert sdf.count('_foliage"') == 18  # 9 plants x 2 sides

    def test_multiple_rows_get_distinct_y_offsets(self):
        sdf = build_world(
            num_rows=2, row_spacing=1.0, row_length=6.0, post_spacing=100.0,
            plant_spacing=100.0, trough_height=1.1, trough_width=0.3,
            post_radius=0.02, half_width=0.3,
        )
        assert 'name="row0_L_trough"' in sdf
        assert 'name="row1_L_trough"' in sdf
        assert '<pose>3.000 0.300' in sdf  # row0 L trough centre (y = 0*spacing + half_width)
        assert '<pose>3.000 1.300' in sdf  # row1 L trough centre (y = 1*spacing + half_width)

    def test_thresholds_span_full_aisle_width_when_given(self):
        sdf = build_world(
            num_rows=1, row_spacing=1.0, row_length=6.0, post_spacing=100.0,
            plant_spacing=100.0, trough_height=1.1, trough_width=0.3,
            post_radius=0.02, half_width=0.3, thresholds=[2.0],
        )
        assert 'name="row0_threshold0"' in sdf
        assert '<size>0.030 0.600' in sdf  # width = 2 * half_width

    def test_no_thresholds_by_default(self):
        sdf = build_world(
            num_rows=1, row_spacing=1.0, row_length=6.0, post_spacing=100.0,
            plant_spacing=100.0, trough_height=1.1, trough_width=0.3,
            post_radius=0.02, half_width=0.3,
        )
        assert 'threshold' not in sdf


class TestMainCollisionGuard:
    """main()'s own collision guard: 2*half_width >= row_spacing must refuse
    to generate, rather than silently emitting overlapping post lines."""

    def test_refuses_when_half_width_would_collide(self, tmp_path, monkeypatch):
        out = tmp_path / 'out.world'
        monkeypatch.setattr(sys, 'argv', [
            'generate_polytunnel_world.py',
            '--row-spacing', '0.5', '--half-width', '0.3',  # 2*0.3 >= 0.5
            '-o', str(out),
        ])
        with pytest.raises(SystemExit):
            main()
        assert not out.exists()

    def test_does_not_refuse_for_a_valid_non_colliding_layout(self, tmp_path, monkeypatch):
        out = tmp_path / 'out.world'
        monkeypatch.setattr(sys, 'argv', [
            'generate_polytunnel_world.py',
            '--num-rows', '1', '--row-spacing', '1.0', '--half-width', '0.2',
            '--row-length', '2.0', '-o', str(out),
        ])
        main()  # must not raise
        assert out.exists()
        content = out.read_text()
        assert '<sdf version="1.7">' in content
        assert 'row0_L_post0' in content
