// Canvas-based top-down 2D map: occupancy grid, robot icon, real driven
// trail, and the assumed/planned path. World coordinates are metres in the
// map frame; screen coordinates are pixels, y-flipped (map +y is up,
// canvas +y is down).

const MAX_TRAIL_POINTS = 5000;

export class Map2D {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');

    this.grid = null;          // { width, height, resolution, originX, originY, originYaw, data }
    this.trail = [];           // real trajectory: [{x,y}, ...]
    this.assumedPath = [];     // assumed trajectory: [{x,y}, ...]
    this.pose = null;          // { x, y, yaw }

    this.pxPerMetre = 40;
    this.viewCenter = { x: 0, y: 0 }; // world coords the canvas is centred on
    this.follow = true;

    this._drag = null;
    canvas.addEventListener('mousedown', (e) => {
      this.follow = false;
      const cb = document.getElementById('map-follow');
      if (cb) cb.checked = false;
      this._drag = { sx: e.clientX, sy: e.clientY, cx: this.viewCenter.x, cy: this.viewCenter.y };
    });
    window.addEventListener('mousemove', (e) => {
      if (!this._drag) return;
      const dx = (e.clientX - this._drag.sx) / this.pxPerMetre;
      const dy = (e.clientY - this._drag.sy) / this.pxPerMetre;
      this.viewCenter = { x: this._drag.cx - dx, y: this._drag.cy + dy };
    });
    window.addEventListener('mouseup', () => { this._drag = null; });
    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.1 : 0.9;
      this.pxPerMetre = Math.min(400, Math.max(4, this.pxPerMetre * factor));
    }, { passive: false });

    this.resize();
  }

  resize() {
    const rect = this.canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = rect.width * dpr;
    this.canvas.height = rect.height * dpr;
    this.canvas.style.width = rect.width + 'px';
    this.canvas.style.height = rect.height + 'px';
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this._cssW = rect.width;
    this._cssH = rect.height;
  }

  setFollow(v) { this.follow = v; }
  clearTrail() { this.trail = []; }

  setOccupancyGrid(msg) {
    this.grid = {
      width: msg.info.width,
      height: msg.info.height,
      resolution: msg.info.resolution,
      originX: msg.info.origin.position.x,
      originY: msg.info.origin.position.y,
      data: msg.data,
    };
  }

  updateRealPose(x, y, yaw) {
    this.pose = { x, y, yaw };
    const last = this.trail[this.trail.length - 1];
    if (!last || Math.hypot(x - last.x, y - last.y) > 0.02) {
      this.trail.push({ x, y });
      if (this.trail.length > MAX_TRAIL_POINTS) this.trail.shift();
    }
    if (this.follow) this.viewCenter = { x, y };
  }

  updateAssumedPath(poses) {
    // poses: array of {x, y} in the map frame, already-ordered
    this.assumedPath = poses;
  }

  worldToScreen(x, y) {
    const cx = this._cssW / 2, cy = this._cssH / 2;
    return {
      sx: cx + (x - this.viewCenter.x) * this.pxPerMetre,
      sy: cy - (y - this.viewCenter.y) * this.pxPerMetre,
    };
  }

  _drawGrid() {
    const ctx = this.ctx;
    const step = 1.0; // 1 metre grid lines
    ctx.strokeStyle = '#182030';
    ctx.lineWidth = 1;
    const halfW = this._cssW / 2 / this.pxPerMetre;
    const halfH = this._cssH / 2 / this.pxPerMetre;
    const x0 = Math.floor((this.viewCenter.x - halfW) / step) * step;
    const x1 = Math.ceil((this.viewCenter.x + halfW) / step) * step;
    const y0 = Math.floor((this.viewCenter.y - halfH) / step) * step;
    const y1 = Math.ceil((this.viewCenter.y + halfH) / step) * step;
    ctx.beginPath();
    for (let x = x0; x <= x1; x += step) {
      const a = this.worldToScreen(x, y0), b = this.worldToScreen(x, y1);
      ctx.moveTo(a.sx, a.sy); ctx.lineTo(b.sx, b.sy);
    }
    for (let y = y0; y <= y1; y += step) {
      const a = this.worldToScreen(x0, y), b = this.worldToScreen(x1, y);
      ctx.moveTo(a.sx, a.sy); ctx.lineTo(b.sx, b.sy);
    }
    ctx.stroke();
  }

  _drawOccupancyGrid() {
    if (!this.grid) return;
    const g = this.grid;
    const ctx = this.ctx;
    const cellPx = Math.max(1, g.resolution * this.pxPerMetre);
    // Cull to the visible world window rather than iterating every cell of
    // a large map every frame.
    const halfW = this._cssW / 2 / this.pxPerMetre;
    const halfH = this._cssH / 2 / this.pxPerMetre;
    const col0 = Math.max(0, Math.floor((this.viewCenter.x - halfW - g.originX) / g.resolution));
    const col1 = Math.min(g.width, Math.ceil((this.viewCenter.x + halfW - g.originX) / g.resolution));
    const row0 = Math.max(0, Math.floor((this.viewCenter.y - halfH - g.originY) / g.resolution));
    const row1 = Math.min(g.height, Math.ceil((this.viewCenter.y + halfH - g.originY) / g.resolution));

    for (let row = row0; row < row1; row++) {
      for (let col = col0; col < col1; col++) {
        const v = g.data[row * g.width + col];
        if (v < 0) continue; // unknown -- leave as background
        const wx = g.originX + (col + 0.5) * g.resolution;
        const wy = g.originY + (row + 0.5) * g.resolution;
        const { sx, sy } = this.worldToScreen(wx, wy);
        const shade = 255 - Math.round((v / 100) * 255);
        ctx.fillStyle = `rgb(${shade * 0.15},${shade * 0.18},${shade * 0.22})`;
        if (v > 65) ctx.fillStyle = '#2a3444';
        ctx.fillRect(sx - cellPx / 2, sy - cellPx / 2, cellPx + 0.5, cellPx + 0.5);
      }
    }
  }

  _drawPolyline(points, color, dashed) {
    if (points.length < 2) return;
    const ctx = this.ctx;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.5;
    ctx.setLineDash(dashed ? [6, 5] : []);
    ctx.beginPath();
    points.forEach((p, i) => {
      const { sx, sy } = this.worldToScreen(p.x, p.y);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  }

  _drawRobot() {
    if (!this.pose) return;
    const { sx, sy } = this.worldToScreen(this.pose.x, this.pose.y);
    const ctx = this.ctx;
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-this.pose.yaw);
    const s = Math.max(8, 0.22 * this.pxPerMetre);
    ctx.fillStyle = '#3fb950';
    ctx.strokeStyle = '#e6edf3';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(s, 0);
    ctx.lineTo(-s * 0.7, s * 0.6);
    ctx.lineTo(-s * 0.4, 0);
    ctx.lineTo(-s * 0.7, -s * 0.6);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  render() {
    const ctx = this.ctx;
    ctx.fillStyle = '#0b0f14';
    ctx.fillRect(0, 0, this._cssW, this._cssH);
    this._drawGrid();
    this._drawOccupancyGrid();
    this._drawPolyline(this.trail, '#3fb950', false);
    this._drawPolyline(this.assumedPath, '#58a6ff', true);
    this._drawRobot();
  }
}
