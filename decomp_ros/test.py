
import rclpy
from rclpy.node import Node

# from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener
# import tf2_geometry_msgs.tf2_geometry_msgs

from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3
from decomp_ros_msgs.msg import Polyhedron
from decomp_ros_msgs.msg import PolyhedronStamped
import random

def toVector3(x,y,z):
    v = Vector3()
    v.x = float(x)
    v.y = float(y)
    v.z = float(z)
    return v
def toPoint3(x,y,z):
    v = Point()
    v.x = float(x)
    v.y = float(y)
    v.z = float(z)
    return v


class TestPublisher(Node):

    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(PolyhedronStamped, 'sfc', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.create_poly();

    def create_poly(self):
        # create a box
        nx = toVector3(1,0,0);
        ny = toVector3(0,1,0);
        nz = toVector3(0,0,1);
        nnx = toVector3(-1,0,0);
        nny = toVector3(0,-1,0);
        nnz = toVector3(0,0,-1);

        px = toPoint3(1,0,0);
        py = toPoint3(0,1,0);
        pz = toPoint3(0,0,1);
        npx = toPoint3(-2,0,0);
        npy = toPoint3(0,-1,0);
        npz = toPoint3(0,0,-1);

        poly = Polyhedron()
        poly.ns.append(nx)
        poly.ps.append(px)
        poly.ns.append(nnx)
        poly.ps.append(npx)
        poly.ns.append(ny)
        poly.ps.append(py)
        poly.ns.append(nny)
        poly.ps.append(npy)
        poly.ns.append(nz)
        poly.ps.append(pz)
        poly.ns.append(nnz)
        poly.ps.append(npz)

        self.poly = PolyhedronStamped()
        self.poly.header.stamp = self.get_clock().now().to_msg()
        self.poly.header.frame_id = "vicon/world"
        self.poly.poly = poly

    def timer_callback(self):
        self.create_poly()
        self.publisher_.publish(self.poly)
        self.get_logger().info('Publishing')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    test_publisher = TestPublisher()

    rclpy.spin(test_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
