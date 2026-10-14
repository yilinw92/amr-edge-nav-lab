"""Measure synthetic projection throughput; never label these Jetson results."""
import json
import math
from pathlib import Path
import platform
import statistics
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from amr_edge_lab.perception import Calibration, process_depth
from amr_edge_lab.perception_numpy import process_depth_numpy

width,height=320,180
# Plane at 2m with deterministic invalid pixels; known input, not camera footage.
values=[math.nan if i%31==0 else 2.0 for i in range(width*height)]
calibration=Calibration(200,200,width/2,height/2)
rows=[]
for backend,stride,voxel in [("python",2,0.08),("numpy",1,0.04),("numpy",2,0.08),("numpy",4,0.08)]:
    times=[]
    for repeat in range(6):
        processor=process_depth if backend=="python" else process_depth_numpy
        start=time.perf_counter();points,stats=processor(values,width,height,calibration,stride=stride,voxel=voxel,min_points=2)
        elapsed=(time.perf_counter()-start)*1000
        if repeat:times.append(elapsed)
    rows.append({"backend":backend,"stride":stride,"voxel_m":voxel,"output_points":len(points),"sampled_pixels":stats["sampled"],"median_ms":round(statistics.median(times),3),"max_of_5_ms":round(max(times),3)})
report={"environment":platform.platform(),"python":platform.python_version(),"input":"synthetic 320x180 depth plane; x86 CPU only","includes":"projection, height gate, voxel centroids; excludes ROS transport and clustering","configurations":rows}
out=Path("artifacts/perception_benchmark.json");out.parent.mkdir(exist_ok=True);out.write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps(report,indent=2))
