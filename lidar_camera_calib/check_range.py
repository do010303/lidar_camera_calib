"""Independent LD06 range check against a plane perpendicular to laser +X."""
import argparse
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from .geometry import scan_points


def forward_distances(scan, half_angle_deg):
    points, _, angles, _ = scan_points(scan)
    wrapped = np.arctan2(np.sin(angles),np.cos(angles))
    return points[np.abs(wrapped)<=np.deg2rad(half_angle_deg),0]


class RangeCheck(Node):
    def __init__(self, args):
        super().__init__('lidar_range_check')
        self.args=args; self.values=[]; self.done=False; self.last=time.monotonic()
        self.create_subscription(LaserScan,args.topic,self.receive,qos_profile_sensor_data)
        self.create_timer(2.,self.watchdog)
        self.get_logger().info(f'Place a stationary flat target perpendicular to laser +X; sector +/-{args.half_angle_deg} deg. Metres. No camera/extrinsic used.')

    def watchdog(self):
        if time.monotonic()-self.last>3:
            self.get_logger().warning('No fresh scan: check driver, topic, DDS domain',throttle_duration_sec=5.)

    def receive(self, scan):
        self.last=time.monotonic()
        x=forward_distances(scan,self.args.half_angle_deg)
        if not len(x):
            self.get_logger().warning('No valid points near +X',throttle_duration_sec=3.)
            return
        value=float(np.median(x));self.values.append(value)
        self.get_logger().info(f'Frame={scan.header.frame_id}; points={len(x)}; plane X={value*100:.2f} cm; sector min/max={x.min()*100:.2f}/{x.max()*100:.2f} cm',throttle_duration_sec=1.)
        if len(self.values)>=self.args.scans:
            v=np.array(self.values)
            print(f'\n{len(v)} scan medians: mean={v.mean()*100:.2f} cm, std={v.std()*1000:.2f} mm, min/max={v.min()*100:.2f}/{v.max()*100:.2f} cm',flush=True)
            if self.args.distance_m is not None:
                print(f'Ruler={self.args.distance_m*100:.2f} cm; mean error={(v.mean()-self.args.distance_m)*1000:+.2f} mm; measured/reference ratio={v.mean()/self.args.distance_m:.4f}',flush=True)
            print('Valid only if sector hits the target and target is perpendicular to +X. No range correction applied.',flush=True)
            self.done=True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic',default='/scan')
    parser.add_argument('--half-angle-deg',type=float,default=2.)
    parser.add_argument('--scans',type=int,default=50)
    parser.add_argument('--distance-m',type=float,help='Actual ruler distance from laser origin to target along +X, metres')
    args,ros_args=parser.parse_known_args()
    if not 0<args.half_angle_deg<30 or args.scans<2 or (args.distance_m is not None and (not np.isfinite(args.distance_m) or args.distance_m<=0)):
        parser.error('Require 0<half-angle-deg<30, scans>=2 and positive finite distance')
    rclpy.init(args=ros_args);node=None
    try:
        node=RangeCheck(args)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=.5)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
