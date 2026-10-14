"""Software velocity gate model. Not a certified emergency-stop system."""
from dataclasses import dataclass
import math

@dataclass
class GateConfig:
    sensor_timeout: float=0.5
    command_timeout: float=0.3
    radius: float=0.2
    margin: float=0.04
    deceleration: float=0.5
    reaction: float=0.15

def swept_clearance(points, linear, angular, config):
    horizon=config.reaction+abs(linear)/config.deceleration
    # Constant initial speed over the entire braking horizon is conservative
    # relative to a decelerating robot; sample at <=2 cm translational spacing.
    steps=max(10,math.ceil(abs(linear)*horizon/0.02))
    result=math.inf
    for i in range(steps+1):
        t=horizon*i/steps
        if abs(angular)<1e-6:x,y=linear*t,0.0
        else:x,y=linear/angular*math.sin(angular*t),linear/angular*(1-math.cos(angular*t))
        for px,py in points: result=min(result,math.hypot(px-x,py-y)-config.radius)
    return result

class VelocityGate:
    def __init__(self, config=None):
        self.config=config or GateConfig();self.stamps={};self.points=[];self.command=(0.0,0.0);self.command_stamp=-math.inf;self.inhibit=False
        c=self.config
        if min(c.sensor_timeout,c.command_timeout,c.radius,c.deceleration)<=0 or c.margin<0 or c.reaction<0:raise ValueError("invalid gate config")
    def observe(self, source, now, points=None):
        self.stamps[source]=now
        if points is not None:self.points=points
    def request(self, linear, angular, now):
        if not all(math.isfinite(v) for v in (linear,angular,now)):raise ValueError("nonfinite command")
        self.command=(linear,angular);self.command_stamp=now
    def evaluate(self, now):
        stale=[s for s in ("scan","depth","odom") if now-self.stamps.get(s,-math.inf)>self.config.sensor_timeout or now<self.stamps.get(s,now)]
        health=not stale;v,w=self.command;clearance=None
        if stale:reason="SENSOR_STALE"
        elif self.inhibit:reason="MISSION_INHIBIT"
        elif now-self.command_stamp>self.config.command_timeout:reason="COMMAND_STALE"
        else:
            clearance=swept_clearance(self.points,v,w,self.config)
            reason="OBSTACLE_STOP" if clearance<self.config.margin+0.01 else "CLEAR"
        allowed=reason=="CLEAR"
        ages={s:round(now-self.stamps[s],4) if s in self.stamps else None for s in ("scan","depth","odom")}
        return {"linear":v if allowed else 0.0,"angular":w if allowed else 0.0,"reason":reason,"sensor_healthy":health,"stale_sources":stale,"source_age_s":ages,"clearance_m":clearance if clearance is not None and math.isfinite(clearance) else None}
