"""Vectorized implementation with the portable algorithm as a differential oracle."""
import math
import numpy as np
from .perception import process_depth

def process_depth_numpy(values,width,height,calibration,stride=2,voxel=0.08,min_points=2,near=0.2,far=5.0,z_min=0.08,z_max=1.2):
    # Reuse parameter validation without processing an image.
    process_depth([float("nan")],1,1,calibration,stride,voxel,min_points,near,far,z_min,z_max)
    if width<=0 or height<=0 or len(values)!=width*height:raise ValueError("invalid depth shape")
    image=np.asarray(values,dtype=np.float64).reshape(height,width)[::stride,::stride]
    v,u=np.mgrid[0:height:stride,0:width:stride]
    valid=np.isfinite(image)&(image>=near)&(image<=far)
    d=image[valid];x=d;y=-(u[valid]-calibration.cx)*d/calibration.fx;z=-(v[valid]-calibration.cy)*d/calibration.fy
    roll,pitch,yaw=calibration.rpy
    cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
    y,z=cr*y-sr*z,sr*y+cr*z;x,z=cp*x+sp*z,-sp*x+cp*z;x,y=cy*x-sy*y,sy*x+cy*y
    points=np.column_stack((x,y,z))+np.asarray(calibration.translation)
    points=points[(points[:,2]>=z_min)&(points[:,2]<=z_max)]
    stats={"sampled":int(image.size),"valid":int(valid.sum()),"height_kept":int(len(points)),"voxels":0,"valid_ratio":float(valid.sum())/max(image.size,1)}
    if len(points)==0:return [],stats
    keys=np.floor(points/voxel).astype(np.int64)
    _,inverse,counts=np.unique(keys,axis=0,return_inverse=True,return_counts=True)
    inverse=inverse.reshape(-1)  # NumPy 2.0/2.1 inverse-shape compatibility.
    sums=np.column_stack([np.bincount(inverse,weights=points[:,i]) for i in range(3)])
    centroids=(sums/counts[:,None])[counts>=min_points]
    stats["voxels"]=len(centroids)
    return [tuple(p) for p in centroids.tolist()],stats
