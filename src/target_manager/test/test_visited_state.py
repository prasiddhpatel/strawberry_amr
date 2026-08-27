"""
Real, executable unit tests for visited_state.py -- pure functions, no
rclpy dependency.

Run: cd src/target_manager && python3 -m pytest test/test_visited_state.py -v
"""
import sys
import os
import json
import tempfile
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'target_manager'))

from visited_state import (  # noqa: E402
    csv_fingerprint, restore_visited_keys, build_sidecar_data, visited_state_path,
)


class TestCsvFingerprint:
    def test_existing_file_returns_mtime_and_size(self):
        with tempfile.NamedTemporaryFile(suffix='.csv', delete=False) as f:
            f.write(b"x,y,z\n1.0,2.0,0.5\n")
            path = f.name
        try:
            fp = csv_fingerprint(path)
            assert isinstance(fp, list) and len(fp) == 2
            assert fp[1] == os.path.getsize(path)
        finally:
            os.unlink(path)

    def test_missing_file_returns_none(self):
        assert csv_fingerprint('/nonexistent/path/does_not_exist.csv') is None

    def test_different_content_gives_different_fingerprint(self):
        with tempfile.NamedTemporaryFile(suffix='.csv', delete=False) as f:
            f.write(b"x,y,z\n1.0,2.0,0.5\n")
            path = f.name
        try:
            fp1 = csv_fingerprint(path)
            with open(path, 'a') as f2:
                f2.write("3.0,4.0,0.5\n")
            fp2 = csv_fingerprint(path)
            assert fp1 != fp2   # size changed at minimum
        finally:
            os.unlink(path)


class TestVisitedStatePath:
    def test_derives_sidecar_name_from_csv_path(self):
        assert visited_state_path('/a/b/semantic_targets.csv') == '/a/b/semantic_targets.visited.json'

    def test_handles_path_without_extension(self):
        assert visited_state_path('/a/b/targets') == '/a/b/targets.visited.json'


class TestRestoreVisitedKeys:
    def test_matching_fingerprint_restores_keys(self):
        fp = [123.0, 456]
        data = {'csv_fingerprint': fp, 'visited_keys': [[6, 13], [20, 13]]}
        visited, was_stale = restore_visited_keys(data, fp, {(6, 13), (20, 13)})
        assert not was_stale
        assert visited == {(6, 13), (20, 13)}

    def test_mismatched_fingerprint_is_stale_and_empty(self):
        data = {'csv_fingerprint': [123.0, 456], 'visited_keys': [[6, 13]]}
        visited, was_stale = restore_visited_keys(data, [999.0, 111], {(6, 13)})
        assert was_stale
        assert visited == set()

    def test_key_not_in_current_seen_is_dropped_even_with_matching_fingerprint(self):
        """A defensive check independent of the staleness one: the
        fingerprint matching only proves 'same CSV file', not that every
        individual sidecar key is one of the actually-loaded plants (e.g.
        a hand-edited or corrupted sidecar)."""
        fp = [123.0, 456]
        data = {'csv_fingerprint': fp, 'visited_keys': [[6, 13], [99, 99]]}
        visited, was_stale = restore_visited_keys(data, fp, {(6, 13)})
        assert not was_stale
        assert visited == {(6, 13)}   # (99, 99) silently dropped, not an error

    def test_missing_fingerprint_key_in_sidecar_treated_as_stale(self):
        """A sidecar missing its own fingerprint field entirely (e.g. from
        a future format change) must not be treated as trivially matching
        via None == None -- only exercised if current_fingerprint is also
        None, which this test explicitly avoids to catch that edge case."""
        data = {'visited_keys': [[6, 13]]}   # no 'csv_fingerprint' key at all
        visited, was_stale = restore_visited_keys(data, [123.0, 456], {(6, 13)})
        assert was_stale
        assert visited == set()

    def test_empty_visited_keys_list(self):
        fp = [1.0, 2]
        visited, was_stale = restore_visited_keys(
            {'csv_fingerprint': fp, 'visited_keys': []}, fp, {(1, 1)})
        assert not was_stale
        assert visited == set()


class TestBuildSidecarData:
    def test_keys_are_sorted_deterministically(self):
        with tempfile.NamedTemporaryFile(suffix='.csv', delete=False) as f:
            f.write(b"x,y,z\n1.0,2.0,0.5\n")
            path = f.name
        try:
            data = build_sidecar_data(path, {(20, 13), (6, 13), (6, 5)})
            assert data['visited_keys'] == [[6, 5], [6, 13], [20, 13]]
        finally:
            os.unlink(path)

    def test_round_trip_through_json_matches_restore(self):
        """The actual end-to-end path the node uses: build -> json.dumps ->
        json.loads -> restore, must recover the exact same set."""
        with tempfile.NamedTemporaryFile(suffix='.csv', delete=False) as f:
            f.write(b"x,y,z\n1.0,2.0,0.5\n3.0,4.0,0.5\n")
            path = f.name
        try:
            original = {(6, 13), (20, 26)}
            data = build_sidecar_data(path, original)
            round_tripped = json.loads(json.dumps(data))
            restored, was_stale = restore_visited_keys(
                round_tripped, csv_fingerprint(path), original)
            assert not was_stale
            assert restored == original
        finally:
            os.unlink(path)
