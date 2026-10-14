import math
import struct
import unittest
from amr_edge_lab.perception import Calibration, decode_depth, process_depth, cluster_obstacles
from amr_edge_lab.safety import VelocityGate
from amr_edge_lab.supervisor import RouteSupervisor

class PerceptionTests(unittest.TestCase):
    def test_calibration_and_mount_rotation(self):
        c=Calibration(100,100,0,0)
        self.assertEqual(c.transform(100,0,2),(2,-2,0.35))
        c=Calibration(100,100,0,0,(0,0,1),(0,0,math.pi/2))
        x,y,z=c.transform(0,0,2)
        self.assertAlmostEqual(x,0);self.assertAlmostEqual(y,2);self.assertEqual(z,1)
    def test_padding_and_endian(self):
        for endian in (False,True):
            data=struct.pack((">" if endian else "<")+"ff",1,2)+b"pad!"
            self.assertEqual(decode_depth(data,2,1,12,endian),[1,2])
        with self.assertRaises(ValueError):decode_depth(b"123",1,1,4)
    def test_voxel_centroids_and_outlier_filter(self):
        p,stats=process_depth([2,2.01,math.nan,4],4,1,Calibration(100,100,0,0),stride=1,voxel=0.5,min_points=2)
        # Negative Y at adjacent pixels puts points in neighboring voxels.
        self.assertEqual(stats["valid"],3)
        p,stats=process_depth([2,2.01,math.inf,0],4,1,Calibration(100,100,2,0),stride=1,voxel=0.5,min_points=2)
        self.assertEqual(len(p),1);self.assertAlmostEqual(p[0][0],2.005)
        self.assertEqual(stats["valid_ratio"],0.5)
    def test_floor_filter_and_clusters(self):
        p,stats=process_depth([2]*4,1,4,Calibration(1,1,0,0),stride=1)
        self.assertEqual(stats["height_kept"],1)
        clusters=cluster_obstacles([(1,1,0.4),(1.1,1,0.5),(1.2,1,0.5),(4,4,1)],minimum=3)
        self.assertEqual(len(clusters),1);self.assertEqual(clusters[0]["points"],3)
    def test_nonfinite_intrinsics(self):
        with self.assertRaises(ValueError):process_depth([1],1,1,Calibration(math.nan,1,0,0))
    def test_numpy_matches_portable_oracle(self):
        try:from amr_edge_lab.perception_numpy import process_depth_numpy
        except ImportError:self.skipTest("optional NumPy not installed")
        import random
        rng=random.Random(2026)
        for trial in range(20):
            values=[rng.uniform(0.05,6) if rng.random()>0.1 else math.nan for _ in range(48*32)]
            c=Calibration(35,36,23.5,15.5,(0.1,-0.05,0.4),(0.02,-0.1,0.15))
            options={"stride":1+trial%3,"voxel":0.13,"min_points":1+trial%2}
            a,sa=process_depth(values,48,32,c,**options);b,sb=process_depth_numpy(values,48,32,c,**options)
            self.assertEqual(sa,sb);self.assertEqual(len(a),len(b))
            for p,q in zip(a,b):
                for x,y in zip(p,q):self.assertAlmostEqual(x,y,places=10)

class SafetyTests(unittest.TestCase):
    def ready(self,t=1):
        g=VelocityGate()
        for s in ("scan","depth","odom"):g.observe(s,t)
        g.request(0.3,0,t);return g
    def test_dropout_zeroes_both_axes(self):
        g=self.ready();g.request(0.3,0.2,1.4);g.observe("scan",1.6);g.observe("odom",1.6)
        r=g.evaluate(1.6);self.assertEqual(r["reason"],"SENSOR_STALE");self.assertEqual((r["linear"],r["angular"]),(0,0));self.assertEqual(r["stale_sources"],["depth"])
    def test_command_watchdog_and_inhibit(self):
        g=self.ready();self.assertEqual(g.evaluate(1.1)["linear"],0.3)
        self.assertEqual(g.evaluate(1.4)["reason"],"COMMAND_STALE")
        g.inhibit=True;self.assertEqual(g.evaluate(1.1)["reason"],"MISSION_INHIBIT")
    def test_braking_envelope_and_reverse(self):
        g=self.ready();g.points=[(0.35,0)]
        self.assertEqual(g.evaluate(1)["reason"],"OBSTACLE_STOP")
        g.request(-0.3,0,1);self.assertEqual(g.evaluate(1)["reason"],"CLEAR")
        g.points=[(-0.35,0)];self.assertEqual(g.evaluate(1)["reason"],"OBSTACLE_STOP")
    def test_nonfinite_and_clock_rollback(self):
        g=self.ready()
        with self.assertRaises(ValueError):g.request(math.nan,0,1)
        self.assertFalse(g.evaluate(0)["sensor_healthy"])

class SupervisorTests(unittest.TestCase):
    def running(self,**kwargs):
        s=RouteSupervisor(**kwargs);s.begin([(1,1),(2,1)],0);token,goal=s.step(True,0);s.accepted(token,True,0.1);return s,token
    def test_multi_goal_and_stale_callbacks(self):
        s,t=self.running();s.result(t,"succeeded",1);self.assertEqual(s.index,1)
        self.assertIsNone(s.step(True,1));u,_=s.step(True,2.1)
        s.result(t,"failed",2.2);self.assertEqual(s.state,"WAITING_FOR_ACCEPTANCE")
        s.accepted(u,True,2.3);s.result(u,"succeeded",3);self.assertEqual(s.state,"SUCCEEDED")
    def test_cancel_before_acceptance_and_no_early_retry(self):
        s=RouteSupervisor();s.begin([(1,1)],0);t,_=s.step(True,0)
        s.step(False,0.2);s.accepted(t,True,0.3)
        self.assertIsNone(s.step(True,2));self.assertEqual(s.state,"CANCELING")
        s.result(t,"canceled",2.1);self.assertEqual(s.state,"PAUSED")
        self.assertIsNone(s.step(True,2.2));self.assertIsNotNone(s.step(True,3.3))
    def test_operator_cancel_never_resumes_even_success_race(self):
        s,t=self.running();s.cancel("operator",0.2);s.result(t,"succeeded",0.3)
        self.assertEqual(s.state,"CANCELED");self.assertIsNone(s.step(True,10))
    def test_retry_budget(self):
        s,t=self.running(max_retries=1);s.result(t,"failed",1);s.step(True,1.1);u,_=s.step(True,2.2);s.accepted(u,False,2.3);self.assertEqual(s.state,"FAILED")
    def test_lost_cancel_result_latches_fault(self):
        s,t=self.running();s.step(False,1);s.step(True,12);self.assertEqual(s.state,"FAULT")
        with self.assertRaises(RuntimeError):s.begin([(1,1)],13)
        self.assertIsNone(s.step(True,14))
    def test_timeout_waits_for_result(self):
        s,t=self.running(timeout=1);s.step(True,1.2);self.assertEqual(s.state,"CANCELING")
        s.result(t,"canceled",1.3);self.assertEqual(s.state,"FAILED")

if __name__=="__main__":unittest.main()
