"""Add numerical readouts to a self-contained Meshcat recording."""
from functools import partial
from http.server import BaseHTTPRequestHandler, SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from threading import Thread

import numpy as np


def write_replay(meshcat_html, result, output):
    """Keep all values on the same simulation clock as the 3D animation."""
    state = result["state"]
    spin_inertia = result["model"].cube.flywheel.inertia
    # theta, gamma and their rates are the actual Drake coordinates.
    cmg = spin_inertia * state[:, 5] * state[:, 4] * np.sin(state[:, 1])
    rows = np.column_stack([
        result["t"], state[:, 5]*60/(2*np.pi),
        np.rad2deg(state[:, 1]-result.get("gimbal_display_offset", 0.0)), np.rad2deg(state[:, 0]),
        np.rad2deg(state[:, 4]), np.rad2deg(state[:, 3]),
        result["torques"][:, 0], result["torques"][:, 1], cmg,
    ])
    angle_note = ("Gimbal angles use the original analytical zero (scene angle minus 90°)."
                  if result.get("gimbal_display_offset", 0.0) else "Gimbal zero: flywheel disc in XZ.")
    fields = [
        ["Time", "s", 3], ["Flywheel", "RPM", 1],
        ["Gimbal angle", "°", 2], ["Cube angle", "°", 2],
        ["Gimbal speed", "°/s", 2], ["Cube speed", "°/s", 2],
        ["Gimbal torque", "N·m", 6], ["Spin motor torque", "N·m", 6],
        ["CMG contribution", "N·m", 5],
    ]
    output.write_text(_render_panel(meshcat_html, rows, fields, "Cube &amp; flywheel",
                 "Recorded simulation · fixed edge", angle_note,
                 "CMG is the gyroscopic contribution about the cube edge, not the net torque."))


def scene_replay_html(meshcat_html, times, angles, rates, gimbal_angles, gimbal_rates,
                      torques, target, speed_limit, cube_angles, contact_forces):
    """Controller readouts on the same clock as the in-memory scene replay."""
    peak_speed = np.maximum.accumulate(np.abs(gimbal_rates))
    rows = np.column_stack([
        times, np.rad2deg(angles), rates * 60 / (2 * np.pi),
        np.rad2deg(gimbal_angles) - 90, np.rad2deg(gimbal_rates),
        np.full(len(times), np.rad2deg(target)), torques,
        peak_speed * 60 / (2 * np.pi),
        np.full(len(times), speed_limit * 60 / (2 * np.pi)),
        (peak_speed > speed_limit).astype(float),
        np.rad2deg(cube_angles), contact_forces,
    ])
    fields = [
        ["Elapsed time", "s", 3], ["Flywheel angle", "°", 2], ["Flywheel speed", "RPM", 2],
        ["Gimbal angle", "°", 2], ["Gimbal speed", "°/s", 2], ["Gimbal target", "°", 2],
        ["Applied torque", "N·m", 4], ["Peak gimbal speed", "RPM", 2],
        ["Speed limit", "RPM", 2], ["Limit exceeded (0=no, 1=yes)", "", 0],
        ["Cube angle", "°", 2], ["Floor normal force", "N", 2],
    ]
    return _render_panel(meshcat_html, rows, fields, "Plate &amp; flywheel",
                 "Recorded simulation · planar floor contact", "Gimbal angles use the analytical zero (scene minus 90°).",
                 "Speed is monitored, not clamped. Peak speed is sampled every 1 ms.")


def _render_panel(meshcat_html, rows, fields, title, subtitle, angle_note, note):
    if len(rows) == 0 or not np.isfinite(rows).all() or np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError("Telemetry must be finite, nonempty and ordered by time.")
    payload = json.dumps(rows.tolist(), separators=(",", ":"), allow_nan=False)
    panel = (PANEL.replace("__TELEMETRY_ROWS__", payload)
             .replace("__FIELDS__", json.dumps(fields))
             .replace("__TITLE__", title).replace("__SUBTITLE__", subtitle)
             .replace("__ANGLE_NOTE__", angle_note).replace("__NOTE__", note))
    if "</body>" not in meshcat_html:
        raise ValueError("Expected a Meshcat HTML export.")
    return meshcat_html.replace("</body>", panel + "\n</body>")


def serve_replay(directory):
    """Serve only the generated output folder, on this computer."""
    handler = partial(SimpleHTTPRequestHandler, directory=str(directory.resolve()))
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    Thread(target=server.serve_forever, daemon=True).start()
    return server, f"http://127.0.0.1:{server.server_port}/simulation.html"


def serve_scene_replay(html):
    """Serve the scene recording from memory without writing any files."""
    content = html.encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path not in ("/", "/simulation.html"):
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    Thread(target=server.serve_forever, daemon=True).start()
    return server, f"http://127.0.0.1:{server.server_port}/simulation.html"


PANEL = r'''
<style>
#telemetry { position:fixed; top:20px; left:20px; z-index:10; width:285px;
  padding:20px; box-sizing:border-box; border-radius:12px; background:rgba(18,27,40,.94);
  color:#f1f5f9; font:14px system-ui,sans-serif; box-shadow:0 5px 24px #0004; }
#telemetry h1 {font-size:18px; margin:0 0 4px;}
#telemetry .muted {color:#a9bacd; font-size:12px; line-height:1.5;}
#telemetry dl {margin:14px 0;}
#telemetry .reading {display:flex; justify-content:space-between; gap:8px; padding:7px 0;
  border-bottom:1px solid #ffffff15;}
#telemetry dt {color:#c4d1df;}
#telemetry dd {margin:0; font-variant-numeric:tabular-nums; white-space:nowrap;}
#telemetry .buttons {display:flex; gap:8px; margin-top:12px;}
#telemetry button {background:#2e425a; color:white; border:1px solid #54708c;
  border-radius:6px; padding:7px 12px; cursor:pointer;}
#telemetry input[type=range] {width:100%; margin:12px 0; accent-color:#58b7ff;}
@media(max-width:650px) {#telemetry {top:8px; left:8px; width:235px; padding:12px; font-size:12px;}}
</style>
<aside id="telemetry" aria-label="Simulation readouts">
  <h1>__TITLE__</h1>
  <div class="muted">__SUBTITLE__</div>
  <div class="muted">__ANGLE_NOTE__</div>
  <dl id="readings"></dl>
  <div class="muted">__NOTE__</div>
  <div class="buttons"><button id="replay-toggle">Pause</button><button id="replay-restart">Restart</button></div>
  <label for="replay-time" class="muted">Playback time</label>
  <input id="replay-time" type="range" min="0" step="0.001" value="0">
  <div id="replay-status" class="muted">Loading recording…</div>
</aside>
<script>
(() => {
  const rows = __TELEMETRY_ROWS__;
  const fields = __FIELDS__;
  const readings = document.getElementById("readings");
  const displays = fields.map(([label]) => {
    const row = document.createElement("div"); row.className = "reading";
    const name = document.createElement("dt"); name.textContent = label;
    const value = document.createElement("dd"); value.textContent = "—";
    row.append(name, value); readings.append(row); return value;
  });
  const slider = document.getElementById("replay-time");
  const toggle = document.getElementById("replay-toggle");
  const status = document.getElementById("replay-status");
  const end = rows[rows.length-1][0];
  slider.max = end;

  function sample(time) {
    let low = 0, high = rows.length-1;
    while (high-low > 1) {
      const middle = Math.floor((low+high)/2);
      if (rows[middle][0] <= time) low = middle; else high = middle;
    }
    const a = rows[low], b = rows[high];
    const fraction = a === b ? 0 : Math.max(0, Math.min(1, (time-a[0])/(b[0]-a[0])));
    return a.map((value, i) => value + fraction*(b[i]-value));
  }

  // Use Meshcat's animation clock, not wall time, so pause/seek/repeat stay in sync.
  function animator() {
    return typeof viewer === "undefined" ? null : viewer.animator;
  }
  toggle.onclick = () => {
    const a = animator(); if (!a) return;
    if (a.playing) a.pause(); else a.play();
  };
  document.getElementById("replay-restart").onclick = () => {
    const a = animator(); if (a) {a.reset(); a.play();}
  };
  slider.oninput = () => {
    const a = animator(); if (!a) return;
    a.pause(); a.seek(Number(slider.value)); a.display_progress(Number(slider.value));
  };
  function update() {
    const a = animator();
    if (a && a.actions.length) {
      const time = Math.max(0, Math.min(end, a.time));
      const values = sample(time);
      displays.forEach((display, i) => {
        const [, unit, digits] = fields[i];
        const value = Math.abs(values[i]) < 0.5*10**(-digits) ? 0 : values[i];
        display.textContent = value.toFixed(digits) + " " + unit;
      });
      slider.value = time;
      toggle.textContent = a.playing ? "Pause" : "Play";
      status.textContent = (a.playing ? "Playing" : "Paused") + " · " + end.toFixed(2) + " s recording";
    }
    requestAnimationFrame(update);
  }
  update();
})();
</script>
'''
