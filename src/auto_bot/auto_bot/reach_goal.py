import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
import math
import sys

class robot_go_to_goal(Node):

    def __init__(self):
        super().__init__('goal_node')
        self.vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription=self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        timer_period = 0.2  # seconds
        self.subscription
        self.robot_pose=Point()
        self.goal_pose=Point()
        self.timer = self.create_timer(timer_period, self.go_to_goal)
        self.distance_goal=0
        self.angle_goal=0           
        self.vel_msg=Twist()
        # self.angle_offset=0


    def listener_callback(self,data):
        self.robot_pose.x=data.pose.pose.position.x
        self.robot_pose.y=data.pose.pose.position.y
        quaternion=data.pose.pose.orientation
        (roll,pitch,yaw)=self.euler_from_quaternion(quaternion.x,quaternion.y,quaternion.z,quaternion.w)
        self.robot_pose.z=yaw
        

    def go_to_goal(self):
        self.goal_pose.x=float(sys.argv[1])
        self.goal_pose.y=float(sys.argv[2])
        # self.angle_offset=float(sys.argv[3])
        self.distance_goal=math.sqrt(pow((self.goal_pose.x-self.robot_pose.x),2)+pow((self.goal_pose.y-self.robot_pose.y),2))
        self.angle_goal=-1.57-(math.atan2((self.goal_pose.y-self.robot_pose.y),(self.goal_pose.x-self.robot_pose.x)))

        angle_turn= (self.angle_goal)-(self.robot_pose.z)
        if abs(angle_turn)>0.1:
            self.vel_msg.angular.z=angle_turn
            self.vel_msg.linear.x=0.0
        else:
            self.vel_msg.linear.x=self.distance_goal

        msg='DTG={:3f} ATG={:3f} AG:{:3f} PA:{:3f}'.format(self.distance_goal,angle_turn,self.angle_goal,self.robot_pose.z)
        self.get_logger().info(msg)
        self.vel_publisher_.publish(self.vel_msg)


    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians





def main(args=None):
    rclpy.init(args=args)

    goal_node = robot_go_to_goal()

    rclpy.spin(goal_node)

    goal_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()