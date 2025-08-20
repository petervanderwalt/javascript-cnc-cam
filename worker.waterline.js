// worker.js (Waterline + profile mode)
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

function unionContours(contoursScaled) {
  if (!contoursScaled || contoursScaled.length === 0) return [];
  const c = new ClipperLib.Clipper();
  c.AddPaths(contoursScaled, ClipperLib.PolyType.ptSubject, true);
  const sol = new ClipperLib.Paths();
  c.Execute(ClipperLib.ClipType.ctUnion, sol, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero);
  return sol;
}

function offsetContoursScaled(pathsScaled, offsetScaled, joinType = ClipperLib.JoinType.jtRound, endType = ClipperLib.EndType.etClosedPolygon) {
  const simplifiedPaths = ClipperLib.Clipper.SimplifyPolygons(pathsScaled, ClipperLib.PolyFillType.pftEvenOdd);
  const co = new ClipperLib.ClipperOffset();
  co.AddPaths(simplifiedPaths, joinType, endType);
  const sol = new ClipperLib.Paths();
  co.Execute(sol, offsetScaled);
  return sol;
}

// Build a simple rectangular outline from bbox
function generateBBoxOutline(bbox) {
  const { minX, maxX, minY, maxY } = bbox;
  const p0 = { X: Math.round(minX*SCALE), Y: Math.round(minY*SCALE) };
  const p1 = { X: Math.round(maxX*SCALE), Y: Math.round(minY*SCALE) };
  const p2 = { X: Math.round(maxX*SCALE), Y: Math.round(maxY*SCALE) };
  const p3 = { X: Math.round(minX*SCALE), Y: Math.round(maxY*SCALE) };
  // ensure closed path by adding the first point again
  return [[p0,p1,p2,p3,p0]];
}

function clonePaths(paths) {
  return paths.map(p => p.map(pt => ({ X: pt.X, Y: pt.Y })));
}


function generateWaterlinePaths(triangles, triCount, bbox, dia, stepdown, mode, stepover) {
  const radius = dia / 2;
  let zmin = bbox.minZ;
  if (zmin > 0) zmin = 0;
  const zmax = bbox.maxZ;

  const fullOutline = generateBBoxOutline(bbox);
  const layers = [];

  // --- Create layers ---
  for (let z = zmax; z >= zmin - 1e-9; z -= stepdown) {
    const layer = { Z: z, Outline: clonePaths(fullOutline), Vectors: [], Toolpath: [], ShowMask: [] };

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
      const scaledVectors = [];
      for (const c of contours) {
        const path = c.map(p => ({ X: Math.round(p[0]*SCALE), Y: Math.round(p[1]*SCALE) }));
        if (path.length >= 3 && polygonArea2D(path) > MIN_POLY_AREA) scaledVectors.push(path);
      }
      layer.Vectors = scaledVectors;
    }

    layers.push(layer);
    const progressPercent = ((zmax - z) / (zmax - zmin)) * 100;
    self.postMessage({ cmd: 'progress', phase: 1, percent: Math.min(100, Math.round(progressPercent)) });
  }

  // --- Shadow masks (top-down) ---
  let accumulatedMask = [];
  for (let i = layers.length - 1; i >= 0; i--) {
    const currentVectors = layers[i].Vectors;
    layers[i].ShowMask = accumulatedMask.length ? clonePaths(accumulatedMask) : [];
    if (currentVectors.length > 0) {
      const c = new ClipperLib.Clipper();
      c.AddPaths(accumulatedMask, ClipperLib.PolyType.ptSubject, true);
      c.AddPaths(currentVectors, ClipperLib.PolyType.ptClip, true);
      const sol = new ClipperLib.Paths();
      c.Execute(ClipperLib.ClipType.ctUnion, sol, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero);
      accumulatedMask = clonePaths(sol);
    }
    const shadowProgress = ((layers.length - 1 - i) / layers.length) * 100;
    self.postMessage({ cmd: 'progress', phase: 2, percent: Math.min(100, Math.round(shadowProgress)) });
  }

  // --- Toolpaths with shadow masks ---
  const offsetScaled = Math.round(radius * SCALE);
  for (let i = 0; i < layers.length; i++) {
    const layer = layers[i];
    const z = layer.Z;

    // Helper to close a 2D path from Clipper and convert to a flat 3D array
    const closeAndFlatten = (p) => {
        if (!p || p.length < 3) return null;
        const path3D = p.map(pt => [pt.X / SCALE, pt.Y / SCALE, z]);
        path3D.push(path3D[0].slice()); // Close the loop by adding the first point again
        return path3D.flat();
    };

    if (mode === 'rough') {
      const c = new ClipperLib.Clipper();
      c.AddPaths(layer.Outline, ClipperLib.PolyType.ptSubject, true);
      if (layer.Vectors.length > 0) {
        c.AddPaths(layer.Vectors, ClipperLib.PolyType.ptClip, true);
      }
      const pocketArea = new ClipperLib.Paths();
      c.Execute(ClipperLib.ClipType.ctDifference, pocketArea, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero);

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
    } else { // Handles 'waterline' (finishing) and 'profile'
      if (layer.Vectors.length > 0) {
        // First, generate the "ideal" toolpath by offsetting the current layer's vectors outwards.
        const idealPath = offsetContoursScaled(layer.Vectors, offsetScaled, ClipperLib.JoinType.jtRound, ClipperLib.EndType.etClosedPolygon);

        // To prevent collisions, we must clip this ideal path against the geometry of the layer ABOVE,
        // which acts as a "no-go" zone.
        const vectorsAbove = (i > 0) ? layers[i-1].Vectors : [];

        let finalPath = idealPath;

        if (vectorsAbove.length > 0) {
          // If there is geometry on the layer above, subtract its area from our ideal toolpath.
          // This prevents the tool from moving "under" the part and causing a collision.
          const c = new ClipperLib.Clipper();
          c.AddPaths(idealPath, ClipperLib.PolyType.ptSubject, true);
          c.AddPaths(vectorsAbove, ClipperLib.PolyType.ptClip, true);
          const sol = new ClipperLib.Paths();
          c.Execute(ClipperLib.ClipType.ctDifference, sol, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero);
          finalPath = sol;
        }

        // Add the resulting, correctly clipped path(s) to the toolpath list for this layer.
        if (finalPath && finalPath.length) {
          for (const p of finalPath) {
            if (polygonArea2D(p) < MIN_POLY_AREA) continue;
            const flatPath = closeAndFlatten(p);
            if (flatPath) layer.Toolpath.push(flatPath);
          }
        }
      }
    }

    const offsetProgress = ((i + 1) / layers.length) * 100;
    self.postMessage({ cmd: 'progress', phase: 3, percent: Math.min(100, Math.round(offsetProgress)) });
  }
  return layers;
}

self.onmessage = function(e) {
  const msg = e.data;
  if (msg.cmd === 'waterline') {
    const { triangles, triCount, bbox, dia, stepdown, mode, stepover } = msg;
    try {
      const res = generateWaterlinePaths(triangles, triCount, bbox, dia, stepdown, mode, stepover);
      self.postMessage({ cmd: 'waterlineResult', layers: res });
    } catch (err) {
      self.postMessage({ cmd: 'error', message: 'Waterline error: ' + (err && err.message) });
    }
  } else {
    self.postMessage({ cmd:'error', message:'Unknown command ' + msg.cmd });
  }
};
