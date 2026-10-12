import csv
import math
from pathlib import Path as FilePath
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, TransformException


class Trajectory(Node):
    def __init__(self):
        super().__init__('rawseeds_trajectory')
        self.frame = self.declare_parameter('frame','map').value
        target = self.declare_parameter('output_csv','').value
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer,self)
        self.publisher = self.create_publisher(Path,'/trajectory',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.path = Path(); self.path.header.frame_id = self.frame
        self.previous = None; self.stream = None
        if target:
            p = FilePath(target).expanduser(); p.parent.mkdir(parents=True,exist_ok=True)
            self.stream = p.open('x',newline='')
            self.writer = csv.writer(self.stream); self.writer.writerow(['timestamp','x','y','yaw'])
        self.create_timer(0.2,self.tick)

    def tick(self):
        try:
            tr = self.buffer.lookup_transform(self.frame,'base_link',Time())
        except TransformException:
            return
        ns = tr.header.stamp.sec*10**9 + tr.header.stamp.nanosec
        if ns == self.previous:
            return
        if self.previous is not None and ns < self.previous:
            self.get_logger().error('Clock moved backwards. Restart the launch for a new run.')
            return
        self.previous = ns
        p = PoseStamped(); p.header = tr.header
        t,q = tr.transform.translation,tr.transform.rotation
        p.pose.position.x=t.x; p.pose.position.y=t.y; p.pose.position.z=t.z
        p.pose.orientation=q
        self.path.header.stamp=tr.header.stamp
        self.path.poses.append(p)
        if len(self.path.poses)>20000:
            self.path.poses.pop(0)
        self.publisher.publish(self.path)
        if self.stream:
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.writer.writerow([f'{tr.header.stamp.sec}.{tr.header.stamp.nanosec:09d}',t.x,t.y,yaw])
            self.stream.flush()

    def destroy_node(self):
        if self.stream:
            self.stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node=Trajectory()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
