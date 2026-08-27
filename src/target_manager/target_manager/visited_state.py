"""
Pure, rclpy-independent logic for target_manager's persisted visited-state
sidecar file -- separated from target_manager_node.py for the same reason
as command_safety.py, reward.py, and the two *_postprocess.py modules
elsewhere in this workspace: this is the part that can be fully unit-
tested without a live ROS 2 environment, and it is worth getting right
through review -- getting the staleness check wrong in either direction
either loses real progress on a restart, or silently marks a genuinely
unvisited plant as already done.
"""
import os


def csv_fingerprint(csv_path):
    """(mtime, size) of csv_path, or None if it doesn't exist. Cheap,
    no hashing needed -- sufficient to detect "this sidecar was written
    against a DIFFERENT semantic_targets.csv than the one loaded now"."""
    try:
        st = os.stat(csv_path)
        return [st.st_mtime, st.st_size]
    except OSError:
        return None


def restore_visited_keys(sidecar_data, current_fingerprint, current_seen_keys):
    """
    sidecar_data: the parsed JSON dict loaded from the sidecar file, i.e.
      {'csv_fingerprint': [...], 'visited_keys': [[kx, ky], ...]}.
    current_fingerprint: csv_fingerprint() of the CSV actually loaded this run.
    current_seen_keys: the set of (kx, ky) keys actually present in this
      run's loaded plant list (self.seen.keys() in the node).

    Returns (visited_set, was_stale: bool). If the sidecar's own recorded
    fingerprint doesn't match current_fingerprint, the sidecar is
    considered stale and an EMPTY set is returned rather than trusting
    keys that may not correspond to this run's actual plants -- silently
    marking a real, unvisited plant as already visited is a worse failure
    than losing a restart's progress once.

    Keys present in the sidecar but absent from current_seen_keys are
    dropped even when the fingerprint matches -- defensive: the fingerprint
    check catches "wrong CSV entirely", this catches "same CSV, but this
    particular key isn't one of the plants actually loaded" (e.g. a
    corrupted or hand-edited sidecar).
    """
    if sidecar_data.get('csv_fingerprint') != current_fingerprint:
        return set(), True
    restored = {tuple(k) for k in sidecar_data.get('visited_keys', [])}
    return {k for k in restored if k in current_seen_keys}, False


def build_sidecar_data(csv_path, visited_keys):
    """The exact structure written to disk -- keys sorted for a
    deterministic, diff-friendly file."""
    return {
        'csv_fingerprint': csv_fingerprint(csv_path),
        'visited_keys': [list(k) for k in sorted(visited_keys)],
    }


def visited_state_path(csv_path):
    base, _ext = os.path.splitext(csv_path)
    return base + '.visited.json'
