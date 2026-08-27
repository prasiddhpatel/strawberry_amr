import { Map2D } from './map2d.js';
import { View3D } from './view3d.js';
import { CameraView } from './camera.js';
import { decodePointCloud2 } from './pointcloud_decode.js';

// roslib.min.js is a UMD bundle (not an ES module) -- it attaches itself to
// window.ROSLIB when loaded as a plain <script>. Loaded from index.html
// before this module, so it's available as a global here.
/* global ROSLIB */

const ROSBRIDGE_PORT = 9090;
const VIDEO_PORT = 8080;
const CAMERA_TOPIC = '/camera/color/image_raw';

function yawFromQuaternion(q) {
  // Same formula as row_nav_node.py's odom_cb -- kept identical
  // deliberately so the viewer's heading matches what the controller acts
  // on, not a different (if mathematically equivalent) convention.
  return Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

function defaultHost() {
  return window.location.hostname || 'localhost';
}

class App {
  constructor() {
    this.mapView = new Map2D(document.getElementById('map-canvas'));
    this.view3d = new View3D(document.getElementById('view3d-container'));
    this.camView = new CameraView(document.getElementById('cam-img'), document.getElementById('cam-status'));

    this.ros = null;
    this.host = defaultHost();

    this._wireUi();
    this.view3d.start();
    this._renderLoop();
    this.connect(this.host);
  }

  _wireUi() {
    const hostInput = document.getElementById('host-input');
    hostInput.value = this.host;
    document.getElementById('host-connect').addEventListener('click', () => {
      this.connect(hostInput.value.trim() || defaultHost());
    });

    document.getElementById('map-follow').addEventListener('change', (e) => {
      this.mapView.setFollow(e.target.checked);
    });
    document.getElementById('map-clear-trail').addEventListener('click', () => {
      this.mapView.clearTrail();
    });
    document.getElementById('pc3d-follow').addEventListener('change', (e) => {
      this.view3d.setFollow(e.target.checked);
    });

    document.querySelectorAll('.tab-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.tab-btn').forEach((b) => b.classList.remove('active'));
        document.querySelectorAll('.panel').forEach((p) => p.classList.remove('active'));
        btn.classList.add('active');
        document.getElementById(btn.dataset.panel).classList.add('active');
        this.mapView.resize();
        this.view3d.resize();
      });
    });

    window.addEventListener('resize', () => {
      this.mapView.resize();
      this.view3d.resize();
    });
  }

  _setConnStatus(cls, text) {
    const el = document.getElementById('conn-status');
    el.className = cls;
    el.querySelector('.label').textContent = text;
  }

  connect(host) {
    this.host = host;
    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }
    this._setConnStatus('connecting', `connecting to ${host}:${ROSBRIDGE_PORT}…`);

    this.ros = new ROSLIB.Ros({ url: `ws://${host}:${ROSBRIDGE_PORT}` });
    this.ros.on('connection', () => {
      this._setConnStatus('connected', `connected — ${host}`);
      this._subscribeAll();
    });
    this.ros.on('error', () => {
      this._setConnStatus('disconnected', 'connection error');
    });
    this.ros.on('close', () => {
      this._setConnStatus('disconnected', 'disconnected — retrying in 3s');
      setTimeout(() => { if (this.host === host) this.connect(host); }, 3000);
    });

    this.camView.connect(host, VIDEO_PORT, CAMERA_TOPIC);
  }

  _subscribeAll() {
    // Fully-qualified ROS 2 type strings ('pkg/msg/Type'), not the ROS1
    // short form ('pkg/Type'). rosbridge_library's ROS2 build does have a
    // documented fallback that normalizes the short form by inserting
    // '/msg/' automatically (verified against ros_loader.py's
    // _get_interface_class), but there's no reason to depend on a
    // compatibility shim when the correct, unambiguous form costs nothing.
    new ROSLIB.Topic({ ros: this.ros, name: '/map', messageType: 'nav_msgs/msg/OccupancyGrid' })
      .subscribe((msg) => this.mapView.setOccupancyGrid(msg));

    // /r2/robot_pose_map, NOT /odometry/filtered directly: this project's
    // real frame chain is map -> odom (slam_toolbox, corrects drift via
    // scan-matching/loop-closure) -> base_link (EKF, world_frame: odom --
    // see ekf.yaml). /odometry/filtered's pose is in the ODOM frame, not
    // map; treating it as map-frame here would be wrong whenever
    // map->odom is non-identity, which is the normal case once SLAM has
    // corrected any drift at all. map_accumulator_node.py does the real
    // tf2 composition server-side and republishes the already-correct
    // map-frame pose on this topic instead.
    new ROSLIB.Topic({ ros: this.ros, name: '/r2/robot_pose_map', messageType: 'geometry_msgs/msg/PoseStamped' })
      .subscribe((msg) => {
        const p = msg.pose.position;
        const yaw = yawFromQuaternion(msg.pose.orientation);
        this.mapView.updateRealPose(p.x, p.y, yaw);
        this.view3d.updateRobotPose(p.x, p.y, p.z, yaw);
      });

    // Two independent sources of "assumed trajectory": row_nav_node's local
    // corridor projection during row-following, and Nav2's global plan
    // during a planned leg (approach/return). Only one is ever actively
    // publishing at a given moment in the current mission state machine;
    // whichever message arrives most recently wins the display.
    const toXY = (path) => path.poses.map((ps) => ({ x: ps.pose.position.x, y: ps.pose.position.y }));
    new ROSLIB.Topic({ ros: this.ros, name: '/row_nav/assumed_path', messageType: 'nav_msgs/msg/Path' })
      .subscribe((msg) => this.mapView.updateAssumedPath(toXY(msg)));
    new ROSLIB.Topic({ ros: this.ros, name: '/plan', messageType: 'nav_msgs/msg/Path' })
      .subscribe((msg) => this.mapView.updateAssumedPath(toXY(msg)));

    // /r2/map_points, NOT /r2/points: the latter is depth_image_proc's raw
    // per-frame, camera-relative cloud (still published, still what
    // map_accumulator_node.py itself subscribes to) -- this app renders
    // the accumulated, map-frame, voxel-downsampled reconstruction that
    // node builds from it, not the raw per-frame view. Already rate-
    // limited server-side by map_publish_rate_hz (default 1 Hz); no
    // additional client-side throttle needed on top of that.
    new ROSLIB.Topic({
      ros: this.ros, name: '/r2/map_points', messageType: 'sensor_msgs/msg/PointCloud2',
      queue_length: 1, compression: 'none',
    }).subscribe((msg) => {
      const { positions, colors, count } = decodePointCloud2(msg);
      this.view3d.setPointCloud(positions, colors);
      document.getElementById('points-count').textContent = `${count.toLocaleString()} accumulated points`;
    });
  }

  _renderLoop() {
    this.mapView.render();
    requestAnimationFrame(() => this._renderLoop());
  }
}

window.addEventListener('DOMContentLoaded', () => { window.__r2app = new App(); });
