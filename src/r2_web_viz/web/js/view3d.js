import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

export class View3D {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b0f14);

    // Standard three.js Y-up convention, matching pointcloud_decode.js's
    // ROS-map-frame remap (sceneX=x, sceneY=z, sceneZ=-y) -- scene Y really
    // is "up" here now that /r2/map_points is a genuine map-frame cloud,
    // not a camera-relative one. camera.up is the three.js default; set
    // explicitly anyway since a previous version of this file used a
    // Z-up scene and got this wrong for that convention.
    this.camera = new THREE.PerspectiveCamera(60, 1, 0.02, 500);
    this.camera.up.set(0, 1, 0);
    this.camera.position.set(-2, 1.6, 2);
    this.camera.lookAt(0, 0, 0);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;

    // GridHelper already lies in the XZ plane (normal along +Y) by
    // default -- exactly the ground plane in this Y-up scene, no rotation
    // needed (the old Z-up version of this file needed one; this one doesn't).
    const grid = new THREE.GridHelper(10, 20, 0x2a3444, 0x182030);
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.5));

    this.geometry = new THREE.BufferGeometry();
    this.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
    this.geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(0), 3));
    this.material = new THREE.PointsMaterial({ size: 0.012, vertexColors: true, sizeAttenuation: true });
    this.points = new THREE.Points(this.geometry, this.material);
    this.scene.add(this.points);

    // Simple robot marker (a small oriented cone) tracking the real
    // /r2/robot_pose_map pose (see updateRobotPose below) now that the
    // scene is a genuine map-frame reconstruction, not a fixed marker at
    // the origin (that was only correct for the old camera-relative view).
    // ConeGeometry's apex points along +Y by default; rotateZ(-90deg)
    // brings it to point along +X, i.e. "forward" at yaw=0 -- verified
    // against the ROS-yaw-equals-scene-Y-rotation derivation in this
    // file's updateRobotPose comment.
    const robotGeo = new THREE.ConeGeometry(0.08, 0.22, 12);
    robotGeo.rotateZ(-Math.PI / 2);
    this.robotMesh = new THREE.Mesh(robotGeo, new THREE.MeshBasicMaterial({ color: 0x3fb950 }));
    this.scene.add(this.robotMesh);

    this.follow = false;
    this._resizeObserver = new ResizeObserver(() => this.resize());
    this._resizeObserver.observe(container);
    this.resize();
  }

  setFollow(v) { this.follow = v; }

  resize() {
    const w = this.container.clientWidth || 1;
    const h = this.container.clientHeight || 1;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
  }

  setPointCloud(positions, colors) {
    this.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    this.geometry.attributes.position.needsUpdate = true;
    this.geometry.attributes.color.needsUpdate = true;
    this.geometry.computeBoundingSphere();
  }

  updateRobotPose(x, y, z, yaw) {
    // x,y,z,yaw are in the ROS map frame (from /r2/robot_pose_map, a real
    // tf2-composed pose -- see map_accumulator_node.py). Same remap as
    // pointcloud_decode.js: sceneX=x, sceneY=z, sceneZ=-y.
    //
    // Rotation: a ROS yaw rotation (about ROS +Z) maps to a three.js
    // rotation of the SAME signed angle about scene +Y under this remap --
    // derived algebraically (substitute the remap into the standard 2D
    // rotation formulas and compare to three.js's rotation-about-Y
    // formula; they match term-for-term with no sign flip), not assumed.
    this.robotMesh.position.set(x, z, -y);
    this.robotMesh.rotation.y = yaw;
    if (this.follow) {
      this.controls.target.set(x, z, -y);
    }
  }

  start() {
    const loop = () => {
      requestAnimationFrame(loop);
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    };
    loop();
  }
}
