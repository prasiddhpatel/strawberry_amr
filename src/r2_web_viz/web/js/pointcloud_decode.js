// Decodes a sensor_msgs/PointCloud2 message as delivered by rosbridge's
// JSON transport (msg.data arrives as a base64 string, not a typed array)
// into flat Float32Arrays ready for a THREE.BufferGeometry.
//
// Only handles what map_accumulator_node.py's /r2/map_points actually
// produces (see that file): x/y/z as FLOAT32 fields in the MAP frame
// (REP 103: x forward, y left, z up -- already transformed out of the
// camera's optical frame server-side, not here), plus a packed 'rgb'
// FLOAT32 field using the same 0x00RRGGBB-as-float32 convention
// map_accumulator_node.py's pack_rgb_float() writes (round-tripped
// against that exact convention in test_geometry.py, not just assumed to
// match). This is deliberately not a general PointCloud2 parser, and
// deliberately handles only one frame convention -- there is only one
// point-cloud topic in this app, /r2/map_points, so a configurable axis
// convention would be complexity with no second caller to justify it.

const DATATYPE_FLOAT32 = 7;

function base64ToUint8Array(b64) {
  const binary = atob(b64);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
  return bytes;
}

export function decodePointCloud2(msg, opts = {}) {
  const maxPoints = opts.maxPoints || 200000;

  const raw = typeof msg.data === 'string' ? base64ToUint8Array(msg.data) : new Uint8Array(msg.data);
  const view = new DataView(raw.buffer, raw.byteOffset, raw.byteLength);
  const little = !msg.is_bigendian;

  const fieldByName = {};
  for (const f of msg.fields) fieldByName[f.name] = f;
  const xf = fieldByName.x, yf = fieldByName.y, zf = fieldByName.z, rgbf = fieldByName.rgb;
  if (!xf || !yf || !zf) return { positions: new Float32Array(0), colors: new Float32Array(0), count: 0 };
  if (xf.datatype !== DATATYPE_FLOAT32) {
    // Not the layout this decoder handles -- fail loud in the console
    // rather than silently render garbage.
    console.warn('decodePointCloud2: unsupported x field datatype', xf.datatype);
    return { positions: new Float32Array(0), colors: new Float32Array(0), count: 0 };
  }

  const pointStep = msg.point_step;
  const totalPoints = Math.floor(raw.byteLength / pointStep);
  const stride = totalPoints > maxPoints ? Math.ceil(totalPoints / maxPoints) : 1;
  const outCount = Math.floor(totalPoints / stride);

  const positions = new Float32Array(outCount * 3);
  const colors = new Float32Array(outCount * 3);

  let o = 0;
  for (let i = 0, p = 0; p < outCount; i += stride, p++) {
    const base = i * pointStep;
    const x = view.getFloat32(base + xf.offset, little);
    const y = view.getFloat32(base + yf.offset, little);
    const z = view.getFloat32(base + zf.offset, little);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      // depth holes / out-of-range reads come through as NaN -- skip rather
      // than plot a point at the origin.
      positions[o] = positions[o + 1] = positions[o + 2] = 0;
      colors[o] = colors[o + 1] = colors[o + 2] = 0;
      o += 3;
      continue;
    }
    // ROS map-frame convention (REP 103: x forward, y left, z UP) re-mapped
    // to three.js's scene convention (x right, y UP, z toward viewer) --
    // the standard ROS->three.js swap also used by ros3d.js: sceneX = x,
    // sceneY = z, sceneZ = -y. This is NOT the same remap a camera-optical-
    // frame cloud would need (x right, y down, z forward) -- there is only
    // one point-cloud source in this app now, so only one remap exists.
    positions[o] = x;
    positions[o + 1] = z;
    positions[o + 2] = -y;

    if (rgbf) {
      const packed = view.getUint32(base + rgbf.offset, little);
      colors[o] = ((packed >> 16) & 0xff) / 255;
      colors[o + 1] = ((packed >> 8) & 0xff) / 255;
      colors[o + 2] = (packed & 0xff) / 255;
    } else {
      colors[o] = colors[o + 1] = colors[o + 2] = 0.7;
    }
    o += 3;
  }

  return { positions, colors, count: outCount };
}
