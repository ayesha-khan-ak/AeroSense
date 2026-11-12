/* =====================================================================
   AeroSense — aeromap.js
   A small, dependency-free slippy map (Web-Mercator) on a <canvas>.

   * Satellite (Esri World Imagery) or street (OpenStreetMap) tiles when
     the laptop has internet; a dark grid when it doesn't — overlays
     (zones, drone, track) always work, so an offline demo never breaks.
   * Pan (drag), zoom (wheel / buttons / double-click), HiDPI aware.
   * Overlay layers are plain draw callbacks: layer(ctx, map).
   * Optional drag handler for editing geofence vertices / waypoints.
   ===================================================================== */
(function (global) {
  "use strict";

  const TILE = 256;
  const BASEMAPS = {
    satellite: {
      url: (z, x, y) => `https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${z}/${y}/${x}`,
      attribution: "Imagery © Esri, Maxar, Earthstar Geographics", maxZoom: 19,
    },
    streets: {
      url: (z, x, y) => `https://tile.openstreetmap.org/${z}/${x}/${y}.png`,
      attribution: "© OpenStreetMap contributors", maxZoom: 19,
    },
    none: { url: null, attribution: "No basemap", maxZoom: 21 },
  };
  const tileCache = new Map();          // shared between map instances

  function lonToX(lon, ws) { return (lon + 180) / 360 * ws; }
  function latToY(lat, ws) {
    const s = Math.sin(Math.max(-85.05, Math.min(85.05, lat)) * Math.PI / 180);
    return (0.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)) * ws;
  }
  function xToLon(x, ws) { return x / ws * 360 - 180; }
  function yToLat(y, ws) {
    const n = Math.PI - 2 * Math.PI * y / ws;
    return 180 / Math.PI * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)));
  }

  class AeroMap {
    constructor(container, opts = {}) {
      this.el = typeof container === "string" ? document.getElementById(container) : container;
      this.center = opts.center || [0, 0];
      this.zoom = opts.zoom || 16;
      this.minZoom = opts.minZoom || 3;
      this.maxZoom = opts.maxZoom || 20;
      this.basemap = opts.basemap || "satellite";
      this.layers = [];
      this.listeners = {};
      this.dragHandler = null;           // {hitTest(x,y)->obj|null, onDrag(obj,lat,lon), onDragEnd(obj)}
      this.dblClickZoom = true;
      this.tileErrors = 0;
      this.tileLoads = 0;

      this.canvas = document.createElement("canvas");
      this.canvas.className = "aeromap-canvas";
      this.canvas.style.cssText = "position:absolute;inset:0;width:100%;height:100%;touch-action:none;cursor:grab;";
      if (getComputedStyle(this.el).position === "static") this.el.style.position = "relative";
      this.el.appendChild(this.canvas);
      this.ctx = this.canvas.getContext("2d");
      this._bindEvents();
      this._dirty = true;
      new ResizeObserver(() => this._resize()).observe(this.el);
      this._resize();
      const loop = () => { if (this._dirty) this._render(); requestAnimationFrame(loop); };
      requestAnimationFrame(loop);
    }

    // ---------------------------------------------------------- public --
    on(evt, fn) { (this.listeners[evt] = this.listeners[evt] || []).push(fn); return this; }
    _emit(evt, data) { (this.listeners[evt] || []).forEach(fn => fn(data)); }
    addLayer(fn) { this.layers.push(fn); this.requestRender(); return fn; }
    requestRender() { this._dirty = true; }
    setView(center, zoom) {
      this.center = [center[0], center[1]];
      if (zoom !== undefined) this.zoom = Math.max(this.minZoom, Math.min(this.maxZoom, zoom));
      this.requestRender();
    }
    setBasemap(name) { this.basemap = BASEMAPS[name] ? name : "none"; this.requestRender(); }
    zoomBy(d, ax, ay) {
      const before = ax !== undefined ? this.unproject(ax, ay) : null;
      this.zoom = Math.max(this.minZoom, Math.min(this.maxZoom, this.zoom + d));
      if (before) {                      // keep the point under the cursor fixed
        const p = this.project(before[0], before[1]);
        this._panPixels(p.x - ax, p.y - ay);
      }
      this.requestRender();
    }
    worldSize() { return TILE * Math.pow(2, this.zoom); }
    project(lat, lon) {
      const ws = this.worldSize();
      const cx = lonToX(this.center[1], ws), cy = latToY(this.center[0], ws);
      return { x: lonToX(lon, ws) - cx + this.w / 2, y: latToY(lat, ws) - cy + this.h / 2 };
    }
    unproject(x, y) {
      const ws = this.worldSize();
      const cx = lonToX(this.center[1], ws), cy = latToY(this.center[0], ws);
      return [yToLat(cy + y - this.h / 2, ws), xToLon(cx + x - this.w / 2, ws)];
    }
    metersPerPixel(lat = this.center[0]) {
      return 40075016.686 * Math.cos(lat * Math.PI / 180) / this.worldSize();
    }
    fitBounds(points, padPx = 30) {
      if (!points.length) return;
      if (this.w < 50 || this.h < 50) { this._pendingFit = [points, padPx]; return; }   // hidden: fit when shown
      this._pendingFit = null;
      let la0 = 90, la1 = -90, lo0 = 180, lo1 = -180;
      points.forEach(p => { la0 = Math.min(la0, p[0]); la1 = Math.max(la1, p[0]); lo0 = Math.min(lo0, p[1]); lo1 = Math.max(lo1, p[1]); });
      this.center = [(la0 + la1) / 2, (lo0 + lo1) / 2];
      for (let z = this.maxZoom; z >= this.minZoom; z -= 0.25) {
        this.zoom = z;
        const a = this.project(la1, lo0), b = this.project(la0, lo1);
        if (b.x - a.x <= this.w - 2 * padPx && b.y - a.y <= this.h - 2 * padPx) break;
      }
      this.requestRender();
    }

    // -------------------------------------------------------- internals --
    _panPixels(dx, dy) {
      const ws = this.worldSize();
      const cx = lonToX(this.center[1], ws) + dx, cy = latToY(this.center[0], ws) + dy;
      this.center = [yToLat(cy, ws), xToLon(cx, ws)];
      this.requestRender();
    }
    _resize() {
      const r = this.el.getBoundingClientRect();
      this.w = Math.max(1, r.width); this.h = Math.max(1, r.height);
      this.dpr = window.devicePixelRatio || 1;
      this.canvas.width = Math.round(this.w * this.dpr);
      this.canvas.height = Math.round(this.h * this.dpr);
      if (this._pendingFit && this.w >= 50 && this.h >= 50) this.fitBounds(...this._pendingFit);
      this.requestRender();
    }
    _xy(e) { const r = this.canvas.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }
    _bindEvents() {
      const c = this.canvas;
      let down = null, dragObj = null, moved = false;
      c.addEventListener("pointerdown", e => {
        const [x, y] = this._xy(e);
        down = { x, y, cx: x, cy: y }; moved = false;
        dragObj = this.dragHandler ? this.dragHandler.hitTest(x, y) : null;
        c.setPointerCapture(e.pointerId);
        c.style.cursor = dragObj ? "move" : "grabbing";
      });
      c.addEventListener("pointermove", e => {
        const [x, y] = this._xy(e);
        this._emit("mousemove", { x, y, latlng: this.unproject(x, y) });
        if (!down) {
          if (this.dragHandler) c.style.cursor = this.dragHandler.hitTest(x, y) ? "move" : (this.cursor || "grab");
          return;
        }
        if (Math.abs(x - down.x) + Math.abs(y - down.y) > 4) moved = true;
        if (dragObj) {
          const ll = this.unproject(x, y);
          this.dragHandler.onDrag(dragObj, ll[0], ll[1]);
          this.requestRender();
        } else if (moved) {
          this._panPixels(down.cx - x, down.cy - y);
          this._emit("pan", {});
        }
        down.cx = x; down.cy = y;
      });
      const up = e => {
        if (!down) return;
        const [x, y] = this._xy(e);
        if (dragObj && moved && this.dragHandler.onDragEnd) this.dragHandler.onDragEnd(dragObj);
        if (!moved) this._emit("click", { x, y, latlng: this.unproject(x, y), target: dragObj });
        down = null; dragObj = null;
        c.style.cursor = this.cursor || "grab";
      };
      c.addEventListener("pointerup", up);
      c.addEventListener("pointercancel", up);
      c.addEventListener("dblclick", e => {
        const [x, y] = this._xy(e);
        this._emit("dblclick", { x, y, latlng: this.unproject(x, y) });
        if (this.dblClickZoom) this.zoomBy(1, x, y);
        e.preventDefault();
      });
      c.addEventListener("wheel", e => {
        e.preventDefault();
        const [x, y] = this._xy(e);
        this.zoomBy(Math.max(-1, Math.min(1, -e.deltaY / 250)), x, y);
      }, { passive: false });
    }
    _tile(z, x, y) {
      const bm = BASEMAPS[this.basemap];
      const key = `${this.basemap}/${z}/${x}/${y}`;
      let t = tileCache.get(key);
      if (!t) {
        t = { img: new Image(), ok: false, failed: false };
        t.img.crossOrigin = "anonymous";
        t.img.onload = () => { t.ok = true; this.tileLoads++; this.requestRender(); };
        t.img.onerror = () => { t.failed = true; this.tileErrors++; this.requestRender(); };
        t.img.src = bm.url(z, x, y);
        tileCache.set(key, t);
        if (tileCache.size > 600) tileCache.delete(tileCache.keys().next().value);
      }
      return t;
    }
    _render() {
      this._dirty = false;
      const ctx = this.ctx;
      ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      ctx.fillStyle = "#06101a";
      ctx.fillRect(0, 0, this.w, this.h);
      const bm = BASEMAPS[this.basemap];
      let loaded = 0, wanted = 0;
      if (bm.url) {
        const tz = Math.min(bm.maxZoom, Math.max(0, Math.floor(this.zoom)));
        const scale = Math.pow(2, this.zoom - tz);
        const size = TILE * scale;
        const ws = TILE * Math.pow(2, this.zoom);
        const cx = lonToX(this.center[1], ws), cy = latToY(this.center[0], ws);
        const left = cx - this.w / 2, top = cy - this.h / 2;
        const n = Math.pow(2, tz);
        const x0 = Math.floor(left / size), y0 = Math.floor(top / size);
        const x1 = Math.floor((left + this.w) / size), y1 = Math.floor((top + this.h) / size);
        for (let ty = y0; ty <= y1; ty++) {
          if (ty < 0 || ty >= n) continue;
          for (let tx = x0; tx <= x1; tx++) {
            const t = this._tile(tz, ((tx % n) + n) % n, ty);
            wanted++;
            if (t.ok) { loaded++; ctx.drawImage(t.img, tx * size - left, ty * size - top, size + 0.5, size + 0.5); }
          }
        }
      }
      if (!bm.url || loaded < wanted) this._grid(ctx, bm.url && loaded === 0 && this.tileErrors > 0);
      for (const layer of this.layers) {
        ctx.save();
        try { layer(ctx, this); } catch (err) { console.error("map layer error", err); }
        ctx.restore();
      }
      this._scaleBar(ctx);
      ctx.font = "9px Segoe UI, Arial"; ctx.textAlign = "right"; ctx.textBaseline = "bottom";
      const attr = bm.attribution;
      const tw = ctx.measureText(attr).width;
      ctx.fillStyle = "rgba(0,0,0,.55)"; ctx.fillRect(this.w - tw - 8, this.h - 14, tw + 8, 14);
      ctx.fillStyle = "#cfd8e3"; ctx.fillText(attr, this.w - 4, this.h - 2);
      this._emit("render", {});
    }
    _grid(ctx, offline) {
      const mpp = this.metersPerPixel();
      let step = 50; while (step / mpp < 60) step *= 2;
      const o = this.project(this.center[0], this.center[1]);
      const px = step / mpp;
      ctx.strokeStyle = "rgba(40,80,110,.35)"; ctx.lineWidth = 1;
      ctx.beginPath();
      for (let x = o.x % px; x < this.w; x += px) { ctx.moveTo(x, 0); ctx.lineTo(x, this.h); }
      for (let y = o.y % px; y < this.h; y += px) { ctx.moveTo(0, y); ctx.lineTo(this.w, y); }
      ctx.stroke();
      if (offline) {
        ctx.fillStyle = "rgba(255,165,2,.85)"; ctx.font = "10px Segoe UI, Arial"; ctx.textAlign = "center";
        ctx.textBaseline = "bottom";
        ctx.fillText("Satellite imagery offline (no internet) — overlays still live", this.w / 2, this.h - 18);
      }
    }
    _scaleBar(ctx) {
      const mpp = this.metersPerPixel();
      const nice = [5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000];
      let m = nice[0]; for (const v of nice) if (v / mpp <= 110) m = v;
      const px = m / mpp, x = 10, y = this.h - 10;
      ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(x, y - 5); ctx.lineTo(x, y); ctx.lineTo(x + px, y); ctx.lineTo(x + px, y - 5); ctx.stroke();
      ctx.fillStyle = "#fff"; ctx.font = "bold 10px Segoe UI, Arial"; ctx.textAlign = "left"; ctx.textBaseline = "bottom";
      ctx.shadowColor = "#000"; ctx.shadowBlur = 3;
      ctx.fillText(m >= 1000 ? `${m / 1000} km` : `${m} m`, x + 2, y - 4);
      ctx.shadowBlur = 0;
    }
  }

  // ------------------------------------------------------ draw helpers --
  const Draw = {
    polygon(ctx, map, pts, s = {}) {
      if (!pts || pts.length < 2) return;
      ctx.beginPath();
      pts.forEach((p, i) => { const q = map.project(p[0], p[1]); i ? ctx.lineTo(q.x, q.y) : ctx.moveTo(q.x, q.y); });
      if (s.closed !== false) ctx.closePath();
      if (s.fill) { ctx.fillStyle = s.fill; ctx.fill(); }
      ctx.setLineDash(s.dash || []);
      ctx.strokeStyle = s.stroke || "#fff"; ctx.lineWidth = s.width || 2; ctx.stroke();
      ctx.setLineDash([]);
    },
    polyline(ctx, map, pts, s = {}) { Draw.polygon(ctx, map, pts, { ...s, closed: false, fill: null }); },
    dot(ctx, map, lat, lon, s = {}) {
      const q = map.project(lat, lon);
      ctx.beginPath(); ctx.arc(q.x, q.y, s.r || 4, 0, Math.PI * 2);
      ctx.fillStyle = s.fill || "#fff"; ctx.fill();
      if (s.stroke) { ctx.strokeStyle = s.stroke; ctx.lineWidth = s.width || 1.5; ctx.stroke(); }
      return q;
    },
    label(ctx, x, y, text, s = {}) {
      ctx.font = s.font || "bold 10px Segoe UI, Arial";
      const w = ctx.measureText(text).width + 8;
      ctx.textAlign = "left"; ctx.textBaseline = "middle";
      const lx = s.align === "center" ? x - w / 2 : x;
      ctx.fillStyle = s.bg || "rgba(5,12,20,.8)"; ctx.fillRect(lx, y - 8, w, 16);
      if (s.border) { ctx.strokeStyle = s.border; ctx.lineWidth = 1; ctx.strokeRect(lx, y - 8, w, 16); }
      ctx.fillStyle = s.color || "#fff"; ctx.fillText(text, lx + 4, y + 0.5);
    },
    drone(ctx, x, y, heading, color) {
      ctx.save(); ctx.translate(x, y); ctx.rotate((heading || 0) * Math.PI / 180);
      ctx.beginPath(); ctx.arc(0, 0, 15, 0, Math.PI * 2);
      ctx.fillStyle = color + "33"; ctx.fill();
      ctx.beginPath(); ctx.moveTo(0, -12); ctx.lineTo(8, 9); ctx.lineTo(0, 4); ctx.lineTo(-8, 9); ctx.closePath();
      ctx.fillStyle = color; ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5; ctx.fill(); ctx.stroke();
      ctx.restore();
    },
    wedge(ctx, map, lat, lon, bearing, fov, rangeM, s = {}) {
      const c = map.project(lat, lon);
      const r = rangeM / map.metersPerPixel(lat);
      const a0 = (bearing - fov / 2 - 90) * Math.PI / 180, a1 = (bearing + fov / 2 - 90) * Math.PI / 180;
      ctx.beginPath(); ctx.moveTo(c.x, c.y); ctx.arc(c.x, c.y, r, a0, a1); ctx.closePath();
      ctx.fillStyle = s.fill || "rgba(0,242,254,.12)"; ctx.fill();
      ctx.strokeStyle = s.stroke || "rgba(0,242,254,.6)"; ctx.lineWidth = 1; ctx.stroke();
    },
    circleM(ctx, map, lat, lon, radiusM, s = {}) {
      const c = map.project(lat, lon);
      ctx.beginPath(); ctx.arc(c.x, c.y, radiusM / map.metersPerPixel(lat), 0, Math.PI * 2);
      ctx.setLineDash(s.dash || [4, 4]); ctx.strokeStyle = s.stroke || "rgba(255,255,255,.25)";
      ctx.lineWidth = 1; ctx.stroke(); ctx.setLineDash([]);
    },
  };

  global.AeroMap = AeroMap;
  global.AeroDraw = Draw;
})(window);
