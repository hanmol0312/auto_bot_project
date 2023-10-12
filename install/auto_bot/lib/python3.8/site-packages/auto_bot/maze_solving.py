import cv2
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge
from .auto_bot_localization import auto_bot_localizer
from .bot_pathplanning import bot_pathplanner
from .bot_motion_planning import bot_motionplaner
from .bot_mapping import bot_mapper
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import os
import numpy as np




class maze_solver(Node):

    def __init__(self):
        super().__init__('maze_solving_node')
        self.vel_publisher=self.create_publisher(Twist,'/cmd_vel',10)
        
        time_period=0.5
        self.timer=self.create_timer(time_period,self.maze_solving)   
        self.image_sub= self.create_subscription(
            Image,
            '/upper_camera/image_raw',
            self.process_data,
            10)
        self.Bridge=CvBridge()
        self.vel_msg=Twist()
        self.image_sub  # prevent unused variable warning
        self.bot_localizer=auto_bot_localizer()
        self.sat_view=np.zeros((100,100))
        self.bot_mapper=bot_mapper()
        self.bot_pathplanner=bot_pathplanner()
        self.bot_motion_planner=bot_motionplaner()
        self.pose_subscriber=self.create_subscription(Odometry,'/odom',self.bot_motion_planner.get_pose,10)
        self.pose_subscriber

    def process_data(self, data):
        frame=self.Bridge.imgmsg_to_cv2(data,'bgr8')
        self.sat_view=frame
        cv2.imshow("Output",self.sat_view)
        cv2.waitKey(1)

    def maze_solving(self):
        frame_disp=self.sat_view.copy()
        self.bot_localizer.localize_bot(self.sat_view,frame_disp)
        self.bot_mapper.graphify(self.bot_localizer.maze_og)
        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph,self.bot_mapper.Graph.start,self.bot_mapper.Graph.end,self.bot_mapper.maze,method="dijkstra")

        self.vel_publisher.publish(self.vel_msg)     
        bot_loc=self.bot_localizer.loc_car
        path=self.bot_pathplanner.path_to_goal
        self.bot_motion_planner.nav_path(bot_loc,path,self.vel_msg,self.vel_publisher)

        img_shortest_path=self.bot_pathplanner.img_shortest_path
        self.bot_motion_planner.display_control_mechanism_in_action(bot_loc,path,img_shortest_path,self.bot_localizer,frame_disp)
        cv2.imshow("Maze(live)",frame_disp)
        cv2.waitKey(1)


        


def main(args=None):
    rclpy.init(args=args)

    node_obj = maze_solver()

    rclpy.spin(node_obj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()