"""Portable calibrated pinhole projection, voxel filtering and spatial clustering."""
from dataclasses import dataclass
from collections import defaultdict, deque
import math
import struct

@dataclass(frozen=True)
class Calibration:
    fx: float
    fy: float
    cx: float
    cy: float
    translation: tuple = (0.0, 0.0, 0.35)
    rpy: tuple = (0.0, 0.0, 0.0)

    def transform(self, u, v, depth):
        # Optical axes (right, down, forward) -> level body axes (forward,left,up).
        x,y,z = depth, -(u-self.cx)*depth/self.fx, -(v-self.cy)*depth/self.fy
        roll,pitch,yaw=self.rpy
        cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
        y,z=cr*y-sr*z,sr*y+cr*z
        x,z=cp*x+sp*z,-sp*x+cp*z
        x,y=cy*x-sy*y,sy*x+cy*y
        return tuple(a+b for a,b in zip((x,y,z),self.translation))

def decode_depth(data, width, height, step, bigendian=False):
    if width<=0 or height<=0 or step<width*4 or len(data)!=step*height:
        raise ValueError("malformed 32FC1 dimensions/payload")
    fmt=(">" if bigendian else "<")+"f"*width
    return [value for row in range(height) for value in struct.unpack_from(fmt,data,row*step)]

def process_depth(values, width, height, calibration, stride=2, voxel=0.08, min_points=2, near=0.2, far=5.0, z_min=0.08, z_max=1.2):
    numbers=(calibration.fx,calibration.fy,calibration.cx,calibration.cy,*calibration.translation,*calibration.rpy,voxel,near,far,z_min,z_max)
    if not all(math.isfinite(v) for v in numbers) or calibration.fx<=0 or calibration.fy<=0 or voxel<=0 or near<0 or far<=near or z_max<=z_min:
        raise ValueError("invalid calibration/filter parameters")
    if width<=0 or height<=0 or len(values)!=width*height or stride<1 or min_points<1:
        raise ValueError("invalid depth shape/sampling parameters")
    buckets=defaultdict(list); sampled=valid=height_kept=0
    for v in range(0,height,stride):
        for u in range(0,width,stride):
            sampled+=1;d=values[v*width+u]
            if not math.isfinite(d) or not near<=d<=far: continue
            valid+=1;p=calibration.transform(u,v,d)
            if not z_min<=p[2]<=z_max:continue
            height_kept+=1;buckets[tuple(math.floor(x/voxel) for x in p)].append(p)
    points=[]
    for key in sorted(buckets):
        group=buckets[key]
        if len(group)>=min_points: points.append(tuple(sum(p[i] for p in group)/len(group) for i in range(3)))
    return points,{"sampled":sampled,"valid":valid,"height_kept":height_kept,"voxels":len(points),"valid_ratio":valid/max(sampled,1)}

def cluster_obstacles(points, cell_size=0.25, minimum=3):
    """8-connected occupied XY bins; a diagnostic grouping, not object recognition."""
    if cell_size<=0 or minimum<1:raise ValueError("invalid clustering parameters")
    bins=defaultdict(list)
    for p in points:bins[(math.floor(p[0]/cell_size),math.floor(p[1]/cell_size))].append(p)
    remaining=set(bins);clusters=[]
    while remaining:
        seed=min(remaining);remaining.remove(seed);queue=deque([seed]);group=[]
        while queue:
            x,y=queue.popleft();group.extend(bins[(x,y)])
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    n=(x+dx,y+dy)
                    if n in remaining:remaining.remove(n);queue.append(n)
        if len(group)>=minimum:
            clusters.append({"points":len(group),"min":[min(p[i] for p in group) for i in range(3)],"max":[max(p[i] for p in group) for i in range(3)]})
    return clusters
