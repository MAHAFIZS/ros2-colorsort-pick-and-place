import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

L1, L2 = 0.30, 0.25           # link lengths [m]
Z_TRAVEL, Z_PICK = 0.15, 0.01 # vertical travel heights

def clamp(x, lo, hi): return max(lo, min(hi, x))

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.sub = self.create_subscription(Point, 'pick_xy', self.on_target, 10)
        self.pub = self.create_publisher(JointState, 'joint_states', 20)
        self.current = {'z_slide':Z_TRAVEL,'shoulder':0.0,'elbow':0.0,'gripper':0.02}
        self.timer = self.create_timer(0.05, self.hold)
        self.get_logger().info('Waiting for targets on /pick_xy ...')

    def on_target(self, p: Point):
        x, y = float(p.x), float(p.y)
        th1, th2 = self.ik_xy(x, y)

        # simple bin rule by Y (left/center/right)
        if y < -0.05: bin_xy = (0.30, -0.10)
        elif y > 0.05: bin_xy = (0.30, 0.10)
        else:          bin_xy = (0.30, 0.00)

        # sequence: over target → down → close → up → over bin → down → open → up
        self.move_smooth(z=Z_TRAVEL, shoulder=th1, elbow=th2)
        self.move_smooth(z=Z_PICK)
        self.move_smooth(gripper=0.0)     # close
        self.move_smooth(z=Z_TRAVEL)
        th1b, th2b = self.ik_xy(*bin_xy)
        self.move_smooth(shoulder=th1b, elbow=th2b)
        self.move_smooth(z=Z_PICK)
        self.move_smooth(gripper=0.02)    # open
        self.move_smooth(z=Z_TRAVEL)

    def ik_xy(self, x, y):
        c2 = clamp((x*x + y*y - L1*L1 - L2*L2)/(2*L1*L2), -1.0, 1.0)
        s2 = math.sqrt(max(0.0, 1 - c2*c2))
        th2 = math.atan2(s2, c2)
        th1 = math.atan2(y, x) - math.atan2(L2*math.sin(th2), L1 + L2*math.cos(th2))
        return th1, th2

    def move_smooth(self, z=None, shoulder=None, elbow=None, gripper=None, steps=40):
        start = self.current.copy()
        target = self.current.copy()
        if z is not None:        target['z_slide'] = z
        if shoulder is not None: target['shoulder'] = shoulder
        if elbow is not None:    target['elbow'] = elbow
        if gripper is not None:  target['gripper'] = gripper
        for i in range(1, steps+1):
            a=i/steps
            pos = {k: start[k]+a*(target[k]-start[k]) for k in start}
            self.publish_js(pos)
            self.current = pos
            time.sleep(0.02)

    def hold(self):
        self.publish_js(self.current)

    def publish_js(self, pos):
        m = JointState(); m.header.stamp=self.get_clock().now().to_msg()
        m.name=['z_slide','shoulder','elbow','gripper']
        m.position=[pos['z_slide'], pos['shoulder'], pos['elbow'], pos['gripper']]
        self.pub.publish(m)

def main():
    rclpy.init(); rclpy.spin(ArmController()); rclpy.shutdown()
