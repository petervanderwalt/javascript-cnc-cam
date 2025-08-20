// worker.gcode.js

// This function processes a single toolpath segment into G-code lines.
function getGcodeForSegment(seg, options) {
  if (!seg || seg.length === 0) return [];

  const { feed, plunge, safeZ } = options;
  const lines = [];
  const p0 = seg[0];

  // Rapid move to the start of the path
  lines.push(`G0 X${p0.x.toFixed(4)} Y${p0.y.toFixed(4)}`);
  // Plunge down to the cutting depth
  lines.push(`G1 Z${p0.z.toFixed(4)} F${plunge}`);

  // Linearly move through all the points in the segment
  for (let i = 1; i < seg.length; i++) {
    const p = seg[i];
    lines.push(`G1 X${p.x.toFixed(4)} Y${p.y.toFixed(4)} F${feed}`);
  }

  // Retract to safe Z after the path is complete
  lines.push(`G0 Z${safeZ.toFixed(3)}`);
  return lines;
}


self.onmessage = function(e) {
  const msg = e.data;
  if (msg.cmd === 'generate') {
    const { paths, options } = msg;

    try {
      if (!paths || paths.length === 0) {
        self.postMessage({ cmd: 'gcodeResult', gcode: 'G21\nG90\nM30\n; No toolpaths to generate.' });
        return;
      }

      const allLines = [
        'G21 ; Use millimeters',
        'G90 ; Use absolute coordinates',
        'G94 ; Units per minute feed rate',
        `G0 Z${options.safeZ.toFixed(3)} ; Rapid retract to safe Z`
      ];

      const totalPaths = paths.length;
      // Report progress every 250 segments to avoid message flooding
      const reportInterval = 250;

      for (let i = 0; i < totalPaths; i++) {
        const segLines = getGcodeForSegment(paths[i], options);
        allLines.push(...segLines);

        // Send a progress update at the specified interval or on the last item
        if ((i + 1) % reportInterval === 0 || (i + 1) === totalPaths) {
          const percent = Math.round(((i + 1) / totalPaths) * 100);
          self.postMessage({ cmd: 'progress', phase: 1, percent: percent });
        }
      }

      allLines.push('M30 ; End of program');
      const gcode = allLines.join('\n');

      self.postMessage({ cmd: 'gcodeResult', gcode: gcode });

    } catch (err) {
      self.postMessage({ cmd: 'error', message: 'G-code generation failed: ' + (err && err.message) });
    }
  }
};
