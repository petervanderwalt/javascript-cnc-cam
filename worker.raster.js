// worker-raster.js (Raster only)

importScripts('./clipper.js');

const SCALE = 10000;
const MIN_POLY_AREA = 1.0;

function distance2(a, b) {
  const dx = a[0]-b[0], dy = a[1]-b[1];
  return dx*dx + dy*dy;
}

function polygonArea2D(pts) {
  let area = 0;
  for (let i = 0; i < pts.length; i++) {
    const j = (i+1) % pts.length;
    area += pts[i].X * pts[j].Y - pts[j].X * pts[i].Y;
  }
  return Math.abs(area) / 2;
}

function linkSegmentsToContours(segments) {
  const threshold = 1e-6;
  const segs = segments.slice();
  const contours = [];
  while (segs.length) {
    const [s0, e0] = segs.pop();
    const contour = [s0.slice(), e0.slice()];
    let changed = true;
    while (changed) {
      changed = false;
      for (let i = segs.length - 1; i >= 0; i--) {
        const [s, e] = segs[i];
        if (distance2(contour[contour.length-1], s) < threshold*threshold) { contour.push(e.slice()); segs.splice(i,1); changed = true; break; }
        if (distance2(contour[contour.length-1], e) < threshold*threshold) { contour.push(s.slice()); segs.splice(i,1); changed = true; break; }
        if (distance2(contour[0], e) < threshold*threshold) { contour.unshift(s.slice()); segs.splice(i,1); changed = true; break; }
        if (distance2(contour[0], s) < threshold*threshold) { contour.unshift(e.slice()); segs.splice(i,1); changed = true; break; }
      }
    }
    if (contour.length > 2 && distance2(contour[0], contour[contour.length-1]) < threshold*threshold) contour.pop();
    contours.push(contour);
  }
  return contours;
}

function unionContours(contoursScaled) {
  if (!contoursScaled || contoursScaled.length === 0) return [];
  const c = new ClipperLib.Clipper();
  c.AddPaths(contoursScaled, ClipperLib.PolyType.ptSubject, true);
  const sol = new ClipperLib.Paths();
  c.Execute(ClipperLib.ClipType.ctUnion, sol, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero);
  return sol;
}

function offsetContoursScaled(pathsScaled, offsetScaled, joinType = ClipperLib.JoinType.jtRound, endType = ClipperLib.EndType.etClosedPolygon) {
  const co = new ClipperLib.ClipperOffset();
  co.AddPaths(pathsScaled, joinType, endType);
  const sol = new ClipperLib.Paths();
  co.Execute(sol, offsetScaled);
  return sol;
}

// Intersect triangle edges with vertical plane X = x
function intersectTrianglePlaneX(v0, v1, v2, x) {
  const points = [];
  const verts = [v0, v1, v2];
  for (let i = 0; i < 3; i++) {
    const a = verts[i], b = verts[(i + 1) % 3];
    if ((a[0] < x && b[0] > x) || (a[0] > x && b[0] < x)) {
      const t = (x - a[0]) / (b[0] - a[0]);
      const y = a[1] + t * (b[1] - a[1]);
      const z = a[2] + t * (b[2] - a[2]);
      points.push([x, y, z]);
    }
  }
  return points.length === 2 ? points : null;
}

// Intersect triangle edges with vertical plane Y = y
function intersectTrianglePlaneY(v0, v1, v2, y) {
  const points = [];
  const verts = [v0, v1, v2];
  for (let i = 0; i < 3; i++) {
    const a = verts[i], b = verts[(i + 1) % 3];
    if ((a[1] < y && b[1] > y) || (a[1] > y && b[1] < y)) {
      const t = (y - a[1]) / (b[1] - a[1]);
      const x = a[0] + t * (b[0] - a[0]);
      const z = a[2] + t * (b[2] - a[2]);
      points.push([x, y, z]);
    }
  }
  return points.length === 2 ? points : null;
}

function generateRasterPaths(triangles, triCount, bbox, dir, stepover, dia) {
  const radius = dia / 2;
  const min = dir === 'x' ? bbox.minX : bbox.minY;
  const max = dir === 'x' ? bbox.maxX : bbox.maxY;

  const segmentsAll = [];
  for (let pos = min; pos <= max + 1e-9; pos += stepover) {
    const segments = [];
    for (let i = 0; i < triCount; i++) {
      const base = i * 9;
      const v0 = [triangles[base], triangles[base + 1], triangles[base + 2]];
      const v1 = [triangles[base + 3], triangles[base + 4], triangles[base + 5]];
      const v2 = [triangles[base + 6], triangles[base + 7], triangles[base + 8]];

      let seg = null;
      if (dir === 'x') seg = intersectTrianglePlaneX(v0, v1, v2, pos);
      else seg = intersectTrianglePlaneY(v0, v1, v2, pos);

      if (seg) segments.push(seg);
    }
    if (segments.length === 0) continue;

    const contours = linkSegmentsToContours(segments);
    if (!contours || contours.length === 0) continue;

    const scaledPaths = contours.map(c => c.map(p => ({ X: Math.round(p[0] * SCALE), Y: Math.round(p[1] * SCALE) })));

    const unioned = unionContours(scaledPaths);
    if (!unioned || unioned.length === 0) continue;

    const offsetScaled = Math.round(radius * SCALE);
    const offsetPolys = offsetContoursScaled(unioned, offsetScaled, ClipperLib.JoinType.jtRound, ClipperLib.EndType.etClosedPolygon);
    if (!offsetPolys || offsetPolys.length === 0) continue;

    for (const poly of offsetPolys) {
      if (polygonArea2D(poly) < MIN_POLY_AREA) continue;
      const path3D = poly.map(pt => dir === 'x' ? [pos, pt.Y / SCALE, 0] : [pt.X / SCALE, pos, 0]);
      segmentsAll.push(path3D.flat());
    }
  }
  return segmentsAll;
}

self.onmessage = function(e) {
  const msg = e.data;
  if (msg.cmd === 'raster') {
    const { triangles, triCount, bbox, dir, stepover, dia } = msg;
    try {
      const res = generateRasterPaths(triangles, triCount, bbox, dir, stepover, dia);
      self.postMessage({ cmd: 'rasterResult', paths: res });
    } catch (err) {
      self.postMessage({ cmd: 'error', message: 'Raster error: ' + (err && err.message) });
    }
  } else {
    self.postMessage({ cmd:'error', message:'Unknown command ' + msg.cmd });
  }
};
