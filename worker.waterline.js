// worker.js (Waterline mode)
importScripts('./clipper.js'); // keep clipper
const SCALE = 10000;

const MIN_POLY_AREA = 0.5;
const EPS = 1e-9;

function interpOnZ(p1, p2, z) {
  const t = (z - p1[2]) / (p2[2] - p1[2]);
  return [p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]), z];
}

function intersectTrianglePlaneZ(v0, v1, v2, z) {
  const pts = [];
  const verts = [v0, v1, v2];
  for (let i = 0; i < 3; i++) {
    const a = verts[i], b = verts[(i + 1) % 3];
    if ((a[2] < z && b[2] > z) || (a[2] > z && b[2] < z)) {
      pts.push(interpOnZ(a, b, z));
    }
  }
  if (pts.length === 2) return pts;
  return null;
}

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

function offsetContoursScaled(pathsScaled, offsetScaled, joinType = ClipperLib.JoinType.jtRound, endType = ClipperLib.EndType.etClosedPolygon) {
  if (!pathsScaled || pathsScaled.length === 0) return [];
  const simplifiedPaths = ClipperLib.Clipper.SimplifyPolygons(pathsScaled, ClipperLib.PolyFillType.pftEvenOdd);
  const co = new ClipperLib.ClipperOffset();
  co.AddPaths(simplifiedPaths, joinType, endType);
  const sol = new ClipperLib.Paths();
  co.Execute(sol, offsetScaled);
  return sol;
}

function generateBBoxOutline(bbox, radius) {
  const { minX, maxX, minY, maxY } = bbox;
  const p0 = { X: Math.round((minX - (radius*2)) * SCALE), Y: Math.round((minY - (radius*2)) * SCALE) };
  const p1 = { X: Math.round((maxX + (radius*2)) * SCALE), Y: Math.round((minY - (radius*2)) * SCALE) };
  const p2 = { X: Math.round((maxX + (radius*2)) * SCALE), Y: Math.round((maxY + (radius*2)) * SCALE) };
  const p3 = { X: Math.round((minX - (radius*2)) * SCALE), Y: Math.round((maxY + (radius*2)) * SCALE) };
  return [[p0,p1,p2,p3]];
}

function clonePaths(paths) {
  return paths.map(p => p.map(pt => ({ X: pt.X, Y: pt.Y })));
}

// MODIFIED: Function now accepts stockAllow
function generateWaterlinePaths(triangles, triCount, bbox, dia, stepdown, mode, stepover, stockAllow) {
  const radius = dia / 2;
  let zmin = bbox.minZ;
  if (zmin > 0) zmin = 0;
  const zmax = bbox.maxZ;

  const fullOutline = generateBBoxOutline(bbox, radius);
  const layers = [];
  const isFinishing = (mode !== 'rough');

  // --- Phase 1: Slicing Model ---
  for (let z = zmax; z >= zmin - EPS; z -= stepdown) {
    const layer = { Z: z, Outline: clonePaths(fullOutline), Vectors: [], Toolpath: [], CollisionMask: [] };
    const segments = [];
    for (let ti = 0; ti < triCount; ti++) {
      const base = ti * 9;
      const v0 = [triangles[base + 0], triangles[base + 1], triangles[base + 2]];
      const v1 = [triangles[base + 3], triangles[base + 4], triangles[base + 5]];
      const v2 = [triangles[base + 6], triangles[base + 7], triangles[base + 8]];
      const seg = intersectTrianglePlaneZ(v0, v1, v2, z);
      if (seg) segments.push(seg);
    }
    if (segments.length > 0) {
      const contours = linkSegmentsToContours(segments);
      layer.Vectors = contours.map(c => c.map(p => ({ X: Math.round(p[0]*SCALE), Y: Math.round(p[1]*SCALE) })))
                              .filter(path => path.length >= 3 && polygonArea2D(path) > MIN_POLY_AREA);
    }
    layers.push(layer);
    const progressPercent = ((zmax - z) / (zmax - zmin)) * 100;
    self.postMessage({ cmd: 'progress', type: 'waterline', phase: 1, percent: Math.min(100, Math.round(progressPercent)) });
  }

  // --- Phase 2: Generating Collision Masks ---
  let accumulatedShadow = [];
  for (let i = 0; i < layers.length; i++) {
    layers[i].CollisionMask = accumulatedShadow.length ? clonePaths(accumulatedShadow) : [];
    layers[i].ShowMask = layers[i].CollisionMask;
    if (layers[i].Vectors.length > 0) {
      const c = new ClipperLib.Clipper();
      c.AddPaths(accumulatedShadow, ClipperLib.PolyType.ptSubject, true);
      c.AddPaths(layers[i].Vectors, ClipperLib.PolyType.ptClip, true);
      const sol = new ClipperLib.Paths();
      c.Execute(ClipperLib.ClipType.ctUnion, sol, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero);
      accumulatedShadow = sol;
    }
    const shadowProgress = ((i + 1) / layers.length) * 100;
    self.postMessage({ cmd: 'progress', type: 'waterline', phase: 2, percent: Math.min(100, Math.round(shadowProgress)) });
  }

  // --- Phase 3: Generating Toolpaths ---
  const offsetScaled = Math.round(radius * SCALE);
  for (let i = 0; i < layers.length; i++) {
    const layer = layers[i];
    const z = layer.Z;
    const closeAndFlatten = (p) => {
        if (!p || p.length < 2) return null;
        const path3D = p.map(pt => [pt.X / SCALE, pt.Y / SCALE, z]);
        path3D.push(path3D[0].slice());
        return path3D.flat();
    };

    const unionClipper = new ClipperLib.Clipper();
    unionClipper.AddPaths(layer.Vectors, ClipperLib.PolyType.ptSubject, true);
    if (layer.CollisionMask.length > 0) {
      unionClipper.AddPaths(layer.CollisionMask, ClipperLib.PolyType.ptClip, true);
    }
    const totalSolidArea = new ClipperLib.Paths();
    if (layer.Vectors.length > 0 || layer.CollisionMask.length > 0) {
       unionClipper.Execute(ClipperLib.ClipType.ctUnion, totalSolidArea, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero);
    }

    if (isFinishing) {
      if (totalSolidArea.length > 0) {
          const finalPath = offsetContoursScaled(totalSolidArea, offsetScaled, ClipperLib.JoinType.jtRound, ClipperLib.EndType.etClosedPolygon);
          for (const p of finalPath) {
              if (polygonArea2D(p) < MIN_POLY_AREA) continue;
              const flatPath = closeAndFlatten(p);
              if(flatPath) layer.Toolpath.push(flatPath);
          }
      }
    } else {
      // --- ROUGHING LOGIC (MODIFIED to include stock allowance) ---
      let areaToMachine = totalSolidArea;
      // 1. If stock allowance is specified, offset the solid area outwards first.
      if (stockAllow && stockAllow > 0) {
        const stockAllowScaled = Math.round(stockAllow * SCALE);
        areaToMachine = offsetContoursScaled(totalSolidArea, stockAllowScaled, ClipperLib.JoinType.jtRound, ClipperLib.EndType.etClosedPolygon);
      }

      // 2. Define the pocket as the difference between the stock outline and the (now offset) solid shape.
      const pocketClipper = new ClipperLib.Clipper();
      pocketClipper.AddPaths(layer.Outline, ClipperLib.PolyType.ptSubject, true);
      if (areaToMachine.length > 0) {
        pocketClipper.AddPaths(areaToMachine, ClipperLib.PolyType.ptClip, true);
      }

      const pocketArea = new ClipperLib.Paths();
      pocketClipper.Execute(ClipperLib.ClipType.ctDifference, pocketArea, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero);

      // 3. Generate pocketing paths by repeatedly offsetting inwards.
      if (pocketArea.length > 0) {
        let currentPaths = offsetContoursScaled(pocketArea, -offsetScaled, ClipperLib.JoinType.jtRound, ClipperLib.EndType.etClosedPolygon);
        while (currentPaths.length > 0) {
          for (const p of currentPaths) {
            if (polygonArea2D(p) < MIN_POLY_AREA) continue;
            const flatPath = closeAndFlatten(p);
            if(flatPath) layer.Toolpath.push(flatPath);
          }
          if (stepover > 0) {
            const stepoverScaled = Math.round(stepover * SCALE);
            currentPaths = offsetContoursScaled(currentPaths, -stepoverScaled, ClipperLib.JoinType.jtRound, ClipperLib.EndType.etClosedPolygon);
          } else {
            currentPaths = [];
          }
        }
      }
    }

    const toolpathProgress = ((i + 1) / layers.length) * 100;
    self.postMessage({ cmd: 'progress', type: 'waterline', phase: 3, percent: Math.min(100, Math.round(toolpathProgress)) });
  }

  return layers;
}

self.onmessage = function(e) {
  const msg = e.data;
  if (msg.cmd === 'waterline') {
    // MODIFIED: Now receives stockAllow
    const { triangles, triCount, bbox, dia, stepdown, mode, stepover, stockAllow } = msg;
    try {
      const res = generateWaterlinePaths(triangles, triCount, bbox, dia, stepdown, mode, stepover, stockAllow);
      self.postMessage({ cmd: 'waterlineResult', layers: res });
    } catch (err) {
      self.postMessage({ cmd: 'error', message: 'Waterline error: ' + (err && err.message) });
    }
  } else {
    self.postMessage({ cmd:'error', message:'Unknown command ' + msg.cmd });
  }
};
