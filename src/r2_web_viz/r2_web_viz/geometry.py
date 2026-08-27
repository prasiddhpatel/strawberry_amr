"""
Pure, dependency-free math for map_accumulator_node.py: quaternion-to-
rotation-matrix, point transformation, voxel keying, and packed-RGB
encode/decode. No rclpy/tf2 imports here on purpose -- see
command_safety.py for the same separation-of-pure-logic-from-ROS-glue
pattern this mirrors, and test/test_geometry.py for the tests this split
makes possible without a ROS runtime.
"""
import numpy as np


def quat_to_rotmat(x, y, z, w):
    """Standard right-handed quaternion -> 3x3 rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def transform_points(points_xyz, rotmat, translation):
    """points_xyz: (N,3) array in the source frame. rotmat: 3x3. translation:
    length-3. Returns (N,3) in the target frame: R @ p + t for each point."""
    return points_xyz @ rotmat.T + np.asarray(translation)


def voxel_key(x, y, z, voxel_size):
    """Maps a continuous point to its integer voxel-grid cell. Two points
    round to the same key iff they fall in the same voxel_size cube --
    this IS the downsampling: accumulation stores one entry per key."""
    return (round(x / voxel_size), round(y / voxel_size), round(z / voxel_size))


def pack_rgb_float(r, g, b):
    """Packs 8-bit r,g,b into the 0x00RRGGBB uint32-reinterpreted-as-float32
    convention PCL/ROS point clouds use for a single FLOAT32 'rgb' field --
    the SAME convention pointcloud_decode.js's decodePointCloud2() already
    reads (see that file, and its verified round-trip test). Encoder and
    decoder must agree on this or the frontend renders garbage colors."""
    packed = ((int(r) & 0xff) << 16) | ((int(g) & 0xff) << 8) | (int(b) & 0xff)
    return np.frombuffer(np.uint32(packed).tobytes(), dtype=np.float32)[0].item()


def unpack_rgb_uint32(rgb_floats):
    """Inverse of pack_rgb_float, vectorized: (N,) float32 array -> three
    (N,) uint8-range arrays (r, g, b). Used only by tests to round-trip
    pack_rgb_float without needing a real PointCloud2 message."""
    as_uint32 = np.asarray(rgb_floats, dtype=np.float32).view(np.uint32)
    r = (as_uint32 >> 16) & 0xff
    g = (as_uint32 >> 8) & 0xff
    b = as_uint32 & 0xff
    return r, g, b
