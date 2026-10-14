"""Dependency-free grid-route replay; this is not a ROS 2/Nav2 runtime."""
import argparse
import csv
import json
import math
from pathlib import Path

from .core import Mission, State, astar, warehouse


def run(scenario="blocked", output=Path("artifacts/offline")):
    world = warehouse()
    start, goal = (10, 10), (68, 48)
    grid = world.inflated(0.3)
    if scenario == "unreachable":
        wall = {(40, y) for y in range(world.height)}
        world = world.with_obstacles(wall)
        grid = world.inflated(0.3)
    mission = Mission()
    mission.start()
    mission.accepted(True)
    route = astar(grid, start, goal)
    frames, dynamic, replans, distance = [], [], 0, 0.0
    position, tick = start, 0
    if not route:
        mission.finished("failed")
    while route:
        frames.append({"tick": tick, "position": list(position), "path": [list(c) for c in route],
                       "dynamic": [list(c) for c in dynamic], "state": mission.state.value})
        if scenario == "canceled" and tick == 12:
            mission.request_cancel()
            mission.finished("canceled")
            break
        if scenario == "blocked" and tick == 15 and len(route) > 8:
            bx, by = route[min(10, len(route)-1)]
            dynamic = [(bx + dx, by + dy) for dx in range(-3, 4) for dy in range(-3, 4)]
            grid = world.with_obstacles(dynamic).inflated(0.3)
            route = astar(grid, position, goal)
            replans += 1
            if not route:
                mission.finished("failed")
                break
        if len(route) == 1:
            mission.finished("succeeded")
            break
        nxt = route[1]
        distance += math.dist(position, nxt) * world.resolution
        position, route, tick = nxt, route[1:], tick + 1
    frames.append({"tick": tick, "position": list(position), "path": [],
                   "dynamic": [list(c) for c in dynamic], "state": mission.state.value})
    summary = {"mode": "offline_grid_baseline_not_nav2", "scenario": scenario,
               "result": mission.state.value, "replans": replans,
               "path_length_m": round(distance, 3), "frames": len(frames),
               "hardware": "synthetic; no Jetson or ZED used"}
    payload = {"width": world.width, "height": world.height, "occupied": sorted(world.occupied),
               "start": start, "goal": goal, "frames": frames, "summary": summary}
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    with (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["tick", "x_m", "y_m", "state"])
        for frame in frames:
            x, y = world.centre(frame["position"])
            writer.writerow([frame["tick"], x, y, frame["state"]])
    (output / "replay.html").write_text(HTML.replace("__DATA__", json.dumps(payload)).replace("__LABEL__", "Synthetic grid-route replay · Offline A* baseline · No robot hardware"), encoding="utf-8")
    print(json.dumps(summary))
    return summary


HTML = '''<!doctype html><html lang="en"><meta charset="utf-8"><title>AMR Edge Navigation Lab — Replay</title>
<style>body{background:#101827;color:#e8eef7;font:16px system-ui;max-width:1080px;margin:30px auto;padding:0 20px}h1{font-size:30px}p{color:#a7bad2}canvas{background:#19263b;width:100%;border:1px solid #3a4b61;border-radius:12px}button{padding:10px 20px;margin:8px;background:#246bd6;color:white;border:0;border-radius:6px}input{width:60%}pre{white-space:pre-wrap;color:#82dcbd}.legend{font-size:14px}</style>
<h1>AMR Edge Navigation Lab</h1><p>__LABEL__</p>
<canvas id="view" width="960" height="720"></canvas><div><button id="play">Play / pause</button><input id="time" type="range" value="0"><span id="status"></span></div>
<p class="legend">Grey: static map · Orange: introduced obstacle · Cyan: planned route · Blue: robot · Green: goal. Geometry shown in cells; planning uses the configured robot clearance.</p><pre id="metrics"></pre>
<script>const d=__DATA__,c=document.getElementById('view'),ctx=c.getContext('2d'),slider=document.getElementById('time');slider.max=d.frames.length-1;let playing=false;document.getElementById('play').onclick=()=>playing=!playing;
function point(p){return[(p[0]+.5)*c.width/d.width,c.height-(p[1]+.5)*c.height/d.height]}function cell(p,color){ctx.fillStyle=color;ctx.fillRect(p[0]*c.width/d.width,c.height-(p[1]+1)*c.height/d.height,c.width/d.width,c.height/d.height)}
function draw(){let f=d.frames[Number(slider.value)];ctx.clearRect(0,0,c.width,c.height);d.occupied.forEach(p=>cell(p,'#526178'));f.dynamic.forEach(p=>cell(p,'#f5a24a'));ctx.beginPath();f.path.forEach((p,i)=>{let[x,y]=point(p);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.strokeStyle='#54dfe3';ctx.lineWidth=3;ctx.stroke();[[d.goal,'#56dca1',10],[f.position,'#559aff',12]].forEach(([p,col,r])=>{let[x,y]=point(p);ctx.beginPath();ctx.arc(x,y,r,0,Math.PI*2);ctx.fillStyle=col;ctx.fill()});document.getElementById('status').textContent=f.state+' · frame '+slider.value;document.getElementById('metrics').textContent=JSON.stringify(d.summary,null,2)}slider.oninput=draw;setInterval(()=>{if(playing){slider.value=(Number(slider.value)+1)%d.frames.length;draw()}},90);draw();</script></html>'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=["normal", "blocked", "canceled", "unreachable"], default="blocked")
    parser.add_argument("--out", type=Path, default=Path("artifacts/offline"))
    args = parser.parse_args()
    run(args.scenario, args.out)


if __name__ == "__main__":
    main()
