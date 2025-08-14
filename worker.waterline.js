// worker.js (Waterline + Profile + True Roughing Pocketing)
importScripts('./clipper.js'); // keep clipper

// -------------------- Constants --------------------
const SCALE = 10000;
const MIN_POLY_AREA = 0.5; // in model units^2, we'll scale it
const EPS = 1e-9;

// Compute scaled threshold once
const MIN_AREA_SCALED = MIN_POLY_AREA * SCALE * SCALE;

// -------------------- Geometry helpers --------------------
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

// area on **scaled** integer paths
function polygonArea2D(path) {
  let area = 0;
  for (let i = 0; i < path.length; i++) {
    const j = (i+1) % path.length;
    area += path[i].X * path[j].Y - path[j].X * path[i].Y;
  }
  return Math.abs(area) / 2;
}

function linkSegmentsToContours(segments) {
  const threshold2 = 1e-12; // (1e-6)^2
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
        if (distance2(contour[contour.length-1], s) < threshold2) { contour.push(e.slice()); segs.splice(i,1); changed = true; break; }
        if (distance2(contour[contour.length-1], e) < threshold2) { contour.push(s.slice()); segs.splice(i,1); changed = true; break; }
        if (distance2(contour[0], e) < threshold2) { contour.unshift(s.slice()); segs.splice(i,1); changed = true; break; }
        if (distance2(contour[0], s) < threshold2) { contour.unshift(e.slice()); segs.splice(i,1); changed = true; break; }
      }
    }
    if (contour.length > 2 && distance2(contour[0], contour[contour.length-1]) < threshold2) contour.pop();
    contours.push(contour);
  }
  return contours;
}

// -------------------- Clipper helpers --------------------
function scaleToIntPaths(contours) {
  const out = [];
  for (const c of contours) {
    if (c.length < 3) continue;
    const path = c.map(p => ({ X: Math.round(p[0]*SCALE), Y: Math.round(p[1]*SCALE) }));
    if (polygonArea2D(path) > MIN_AREA_SCALED) out.push(path);
  }
  return out;
}

function unionPathsToPolyTree(pathsScaled) {
  const c = new ClipperLib.Clipper();
  if (pathsScaled && pathsScaled.length) c.AddPaths(pathsScaled, ClipperLib.PolyType.ptSubject, true);
  const polytree = new ClipperLib.PolyTree();
  c.Execute(
    ClipperLib.ClipType.ctUnion,
    polytree,
    ClipperLib.PolyFillType.pftNonZero,
    ClipperLib.PolyFillType.pftNonZero
  );
  return polytree;
}

// Flatten a PolyTree to paths with **correct orientation**
// Convention here: outer = CCW (Orientation==true), hole = CW (Orientation==false)
function polyTreeToOrientedPaths(polytree) {
  const result = [];
  function visit(node) {
    for (let i = 0; i < node.Childs.length; i++) {
      const child = node.Childs[i];
      let p = child.Contour.slice();
      const shouldBeCCW = !child.IsHole;
      // ClipperLib.Orientation(path) returns true if path orientation is counter-clockwise
      if (ClipperLib.Orientation(p) !== shouldBeCCW) p.reverse();
      if (p.length >= 3 && polygonArea2D(p) > MIN_AREA_SCALED) result.push(p);
      visit(child);
    }
  }
  visit(polytree);
  return result;
}

// Union arbitrary paths and get a single path set with holes preserved by orientation
function unionPathsOriented(pathsScaledArray) {
  const c = new ClipperLib.Clipper();
  for (const arr of pathsScaledArray) {
    if (arr && arr.length) c.AddPaths(arr, ClipperLib.PolyType.ptSubject, true);
  }
  const polytree = new ClipperLib.PolyTree();
  c.Execute(
    ClipperLib.ClipType.ctUnion,
    polytree,
    ClipperLib.PolyFillType.pftNonZero,
    ClipperLib.PolyFillType.pftNonZero
  );
  return polyTreeToOrientedPaths(polytree);
}

// General offset (delta can be negative for insets). Paths must be oriented.
function offsetContoursScaled(pathsScaled, deltaScaled, joinType = ClipperLib.JoinType.jtRound, endType = ClipperLib.EndType.etClosedPolygon) {
  if (!pathsScaled || !pathsScaled.length) return [];
  const co = new ClipperLib.ClipperOffset();
  co.AddPaths(pathsScaled, joinType, endType);
  const sol = new ClipperLib.Paths();
  co.Execute(sol, deltaScaled);
  // Filter tiny junk
  return sol.filter(p => p.length >= 3 && polygonArea2D(p) > MIN_AREA_SCALED);
}

// Convert scaled 2D paths to flattened 3D at Z
function to3DFlat(pathsScaled, z) {
  const out = [];
  for (const p of pathsScaled) {
    const path3D = p.map(pt => [pt.X / SCALE, pt.Y / SCALE, z]);
    out.push(path3D.flat());
  }
  return out;
}

// -------------------- Pocketing logic --------------------
// Create concentric pocket rings inside "machinable" region (layer + shadow),
// respecting holes. Returns array of flattened 3D paths at Z.
function pocketRings(layerPathsScaled, shadowPathsScaled, dia, stepover, z) {
  const radiusScaled = Math.max(1, Math.round((dia * 0.5) * SCALE));
  const stepoverScaled = Math.max(1, Math.round(stepover * SCALE));

  // 1) Build machinable area with proper holes preserved
  const machinableOriented = unionPathsOriented([layerPathsScaled, shadowPathsScaled]);
  if (!machinableOriented.length) return [];

  // 2) First inset by tool radius to get centerline boundary
  let current = offsetContoursScaled(machinableOriented, -radiusScaled);
  if (!current.length) return [];

  const out3D = [];

  // 3) Emit rings, then repeatedly inset by stepover
  while (current.length) {
    out3D.push(...to3DFlat(current, z));
    current = offsetContoursScaled(current, -stepoverScaled);
  }

  return out3D;
}

// -------------------- Main generator --------------------
let totalProfile = []; // used for "profile" mode accumulation

function generateWaterlinePaths(triangles, triCount, bbox, dia, stepdown, mode, stepoverArg) {
  const zmin = bbox.minZ;
  const zmax = bbox.maxZ;

  // Default stepover if not supplied (45% of dia)
  const stepover = (typeof stepoverArg === 'number' && stepoverArg > 0) ? stepoverArg : (dia * 0.45);

  const layerZs = [];
  const layersScaled = []; // raw per-layer closed polygons (scaled int), no union yet

  // ---- 1/3: Slice model into contours per Z ----
  for (let z = zmin; z <= zmax + 1e-9; z += stepdown) {
    layerZs.push(z);
    const segments = [];

    // Intersect triangles with this Z plane
    for (let ti = 0; ti < triCount; ti++) {
      const base = ti * 9;
      const v0 = [triangles[base+0], triangles[base+1], triangles[base+2]];
      const v1 = [triangles[base+3], triangles[base+4], triangles[base+5]];
      const v2 = [triangles[base+6], triangles[base+7], triangles[base+8]];
      const seg = intersectTrianglePlaneZ(v0, v1, v2, z);
      if (seg) segments.push(seg);
    }

    if (segments.length === 0) {
      layersScaled.push([]);
    } else {
      const contours = linkSegmentsToContours(segments);
      const scaled = scaleToIntPaths(contours);
      layersScaled.push(scaled);
    }

    // progress
    const progressPercent = ((z - zmin) / Math.max(EPS, (zmax - zmin))) * 100;
    self.postMessage({
      cmd: 'progress',
      phase: '1/3: preSlice',
      percent: Math.min(100, Math.round(progressPercent))
    });
  }

  // ---- 2/3: Downwards shadow masks (cumulative union) ----
  const shadowMasks = new Array(layersScaled.length);
  let prevUnionOriented = []; // oriented path set (scaled) of union of layers i..end
  for (let i = layersScaled.length - 1; i >= 0; i--) {
    const currentLayer = layersScaled[i];
    if (!prevUnionOriented.length) {
      prevUnionOriented = unionPathsOriented([currentLayer]);
    } else if (currentLayer.length > 0) {
      prevUnionOriented = unionPathsOriented([prevUnionOriented, currentLayer]);
    }
    shadowMasks[i] = prevUnionOriented;

    const shadowProgress = ((layersScaled.length - 1 - i) / Math.max(1, layersScaled.length)) * 100;
    self.postMessage({ cmd: 'progress', phase: '2/3: shadowMask', percent: Math.min(100, Math.round(shadowProgress)) });
  }

  // ---- 3/3: Generate output toolpaths ----
  const outPaths = [];
  let runningProfile = []; // for profile/rough profile outline accumulation (unchanged behavior)

  for (let i = 0; i < layersScaled.length; i++) {
    const z = layerZs[i];
    const layer = layersScaled[i];
    const shadow = shadowMasks[i] || [];

    if (mode === "rough") {
      // True pocketing between boundary & holes (concentric rings)
      const rings3D = pocketRings(layer, shadow, dia, stepover, z);
      outPaths.push(...rings3D);
    } else {
      // WATERLINE: single offset of (layer ∪ shadow) by tool radius (centerline boundary only)
      const machinableOriented = unionPathsOriented([layer, shadow]);
      const radiusScaled = Math.max(1, Math.round((dia * 0.5) * SCALE));
      if (machinableOriented.length) {
        const onePass = offsetContoursScaled(machinableOriented, -radiusScaled);
        outPaths.push(...to3DFlat(onePass, z));
      }
    }

    // PROFILE support (keep your original intent)
    if (mode === "rough" || mode === "profile") {
      // Accumulate total outer outline to run a per-layer profiling pass
      runningProfile = unionPathsOriented([runningProfile, layer]);
      if (runningProfile.length) {
        const radiusScaled = Math.max(1, Math.round((dia * 0.5) * SCALE));
        const outlinePolys = offsetContoursScaled(runningProfile, -radiusScaled);
        outPaths.push(...to3DFlat(outlinePolys, z));
      }
    }

    const offsetProgress = ((i + 1) / Math.max(1, layersScaled.length)) * 100;
    self.postMessage({ cmd: 'progress', phase: '3/3: offsetPaths', percent: Math.min(100, Math.round(offsetProgress)) });
  }

  return outPaths;
}

// -------------------- Worker message handling --------------------
self.onmessage = function(e) {
  const msg = e.data;
  if (msg.cmd === 'waterline') {
    const { triangles, triCount, bbox, dia, stepdown, mode } = msg;
    // optional stepover (defaults to 45% dia when omitted)
    const stepover = (typeof msg.stepover === 'number') ? msg.stepover : undefined;

    try {
      const res = generateWaterlinePaths(triangles, triCount, bbox, dia, stepdown, mode, stepover);
      self.postMessage({ cmd: 'waterlineResult', paths: res });
    } catch (err) {
      self.postMessage({ cmd: 'error', message: 'Waterline error: ' + (err && err.message) });
    }
  } else {
    self.postMessage({ cmd:'error', message:'Unknown command ' + msg.cmd });
  }
};
