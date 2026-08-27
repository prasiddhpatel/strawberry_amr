// web_video_server's /stream endpoint returns a plain MJPEG multipart HTTP
// response -- a browser <img> tag renders that natively as a live video
// with zero client-side decoding code. No WebSocket, no WebRTC.

export class CameraView {
  constructor(imgEl, statusEl) {
    this.img = imgEl;
    this.status = statusEl;
    this._url = null;
    this.img.addEventListener('error', () => this._setStatus('stream error - retrying', true));
    this.img.addEventListener('load', () => this._setStatus('live'));
  }

  _setStatus(text, isError) {
    if (!this.status) return;
    this.status.textContent = text;
    this.status.style.color = isError ? '#f85149' : '';
  }

  connect(host, videoPort, topic) {
    const url = `http://${host}:${videoPort}/stream?topic=${encodeURIComponent(topic)}&type=mjpeg&quality=80`;
    if (url === this._url) return;
    this._url = url;
    this._setStatus('connecting…');
    this.img.src = url;
  }

  disconnect() {
    this.img.removeAttribute('src');
    this._url = null;
    this._setStatus('disconnected', true);
  }
}
