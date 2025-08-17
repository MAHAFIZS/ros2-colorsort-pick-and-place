import os, cv2, numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

# HSV thresholds (tweak if your sample differs)
HSV_RANGES = {
    'red1':  ((0,120,70),   (10,255,255)),
    'red2':  ((170,120,70), (180,255,255)),
    'green': ((35,80,70),   (85,255,255)),
    'blue':  ((95,80,70),   (130,255,255)),
}

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # Tunable params
        self.declare_parameter('pixels_to_meters', 0.001)  # ~0.6 m over 600 px
        self.declare_parameter('origin_x', 0.05)
        self.declare_parameter('origin_y', 0.0)

        # Default image lives in the package share
        pkg_share = get_package_share_directory('colorsort_arm')
        default_img = os.path.join(pkg_share, 'sample.jpg')
        self.declare_parameter('image_path', default_img)

        self.px2m = float(self.get_parameter('pixels_to_meters').value)
        self.orgx = float(self.get_parameter('origin_x').value)
        self.orgy = float(self.get_parameter('origin_y').value)
        img_path = str(self.get_parameter('image_path').value)

        self.img = cv2.imread(img_path)
        if self.img is None:
            self.get_logger().warn(f'Image not found at {img_path}')
        else:
            self.get_logger().info(f'Using image: {img_path}')

        self.pick_pub = self.create_publisher(Point, 'pick_xy', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'scene_markers', 10)
        self.color_cycle = ['red', 'green', 'blue']
        self.i = 0

        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        if self.img is None:
            return
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        centers = {}
        # red = two ranges
        mask_red = cv2.inRange(hsv, *HSV_RANGES['red1']) | cv2.inRange(hsv, *HSV_RANGES['red2'])
        centers['red'] = self.centroid(mask_red)
        centers['green'] = self.centroid(cv2.inRange(hsv, *HSV_RANGES['green']))
        centers['blue'] = self.centroid(cv2.inRange(hsv, *HSV_RANGES['blue']))

        self.publish_markers(centers)

        # choose next available target by color
        color = self.color_cycle[self.i % len(self.color_cycle)]
        if centers.get(color) is not None:
            u, v = centers[color]
            H, W = self.img.shape[:2]
            x = self.orgx + (u - W/2) * self.px2m
            y = self.orgy + (v - H/2) * self.px2m
            msg = Point(); msg.x = float(x); msg.y = float(y); msg.z = 0.0
            self.pick_pub.publish(msg)
            self.get_logger().info(f'Target {color}: ({msg.x:.3f}, {msg.y:.3f})')
            self.i += 1

    def centroid(self, mask):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] == 0:
            return None
        u = int(M['m10']/M['m00']); v = int(M['m01']/M['m00'])
        return (u, v)

    def publish_markers(self, centers):
        ma = MarkerArray(); mid = 0
        # box markers at detected centers
        if self.img is not None:
            H, W = self.img.shape[:2]
            for color, c in centers.items():
                if c is None:
                    continue
                u, v = c
                x = self.orgx + (u - W/2) * self.px2m
                y = self.orgy + (v - H/2) * self.px2m
                m = Marker()
                m.header.frame_id = 'base'
                m.id = mid; mid += 1
                m.type = Marker.CUBE; m.action = Marker.ADD
                m.pose.position.x = float(x); m.pose.position.y = float(y); m.pose.position.z = 0.0
                m.scale.x = m.scale.y = m.scale.z = 0.04
                m.color.a = 1.0
                if color == 'red':
                    m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
                elif color == 'green':
                    m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
                else:  # blue
                    m.color.r, m.color.g, m.color.b = 0.0, 0.0, 1.0
                ma.markers.append(m)

        # three fixed bins (red, green, blue)
        for idx, (bx, by) in enumerate([(0.30, -0.10), (0.30, 0.00), (0.30, 0.10)]):
            m = Marker()
            m.header.frame_id = 'base'
            m.id = mid; mid += 1
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x = float(bx); m.pose.position.y = float(by); m.pose.position.z = 0.0
            m.scale.x = m.scale.y = 0.06; m.scale.z = 0.02
            m.color.a = 0.3
            if idx == 0:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
            elif idx == 1:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 0.0, 1.0
            ma.markers.append(m)

        self.markers_pub.publish(ma)

def main():
    rclpy.init()
    rclpy.spin(ColorDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

