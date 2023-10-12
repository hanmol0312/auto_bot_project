import cv2
from numpy import interp
from math import pow,atan2,sqrt,degrees,asin
from .import config
import numpy as np

class bot_motionplaner():
    def __init__(self) -> None:
        self.count=0

        self.pt_i_taken=False

        self.init_loc=0

        self.angle_rotation_computed=False

        self.bot_angle_i=0

        self.bot_angle_s=0

        self.bot_angle_rel=0

        self.bot_angle_init=0

        self.goal_not_reached_flag=True
        self.goal_pose_x=0
        self.goal_pose_y=0
        self.path_iter=0




    @staticmethod
    def angle_n_dist(pt_a,pt_b):
        # Trignometric rules Work Considering.... 
        #
        #       [ Simulation/Normal Convention ]      [ Image ]
        #
        #                    Y                    
        #                     |                     
        #                     |___                     ____ 
        #                          X                  |     X
        #                                             |
        #                                           Y
        #
        # Solution: To apply same rules , we subtract the (first) point Y axis with (Second) point Y axis
        error_x = pt_b[0] - pt_a[0]
        error_y = pt_a[1] - pt_b[1]

        # Calculating distance between two points
        distance = sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) )

        # Calculating angle between two points [Output : [-Pi,Pi]]
        angle = atan2(error_y,error_x)
        # Converting angle from radians to degrees
        angle_deg = degrees(angle)

        if (angle_deg>0):
            return (angle_deg),distance
        else:
            # -160 +360 = 200, -180 +360 = 180,  -90 + 360 = 270
            return (angle_deg + 360),distance
        
        #             Angle bw Points 
        #      (OLD)        =>      (NEW) 
        #   [-180,180]             [0,360]


    def get_pose(self,data):
        quaternions=data.pose.pose.orientation
        (roll,pitch,yaw)=self.euler_from_quaternion(quaternions.x,quaternions.y,quaternions.z,quaternions.w)
        yaw_angle=degrees(yaw)

        if(yaw_angle>0):
            self.bot_angle_s=yaw_angle
        else:
            self.bot_angle_s=yaw_angle+360

    @staticmethod
    def bck_to_orig(pt,transform_arr,rot_mat):

        st_col = transform_arr[0] # cols X
        st_row = transform_arr[1] # rows Y
        tot_cols = transform_arr[2] # total_cols / width W
        tot_rows = transform_arr[3] # total_rows / height H
        
        # point --> (col(x),row(y)) XY-Convention For Rotation And Translated To MazeCrop (Origin)
        #pt_array = np.array( [pt[0]+st_col, pt[1]+st_row] )
        pt_array = np.array( [pt[0], pt[1]] )
        
        # Rot Matrix (For Normal XY Convention Around Z axis = [cos0 -sin0]) But for Image convention [ cos0 sin0]
        #                                                      [sin0  cos0]                           [-sin0 cos0]
        rot_center = (rot_mat @ pt_array.T).T# [x,y]
        
        # Translating Origin If neccasary (To get whole image)
        rot_cols = tot_cols#tot_rows
        rot_rows = tot_rows#tot_cols
        rot_center[0] = rot_center[0] + (rot_cols * (rot_center[0]<0) ) + st_col  
        rot_center[1] = rot_center[1] + (rot_rows * (rot_center[1]<0) ) + st_row 
        return rot_center

    def display_control_mechanism_in_action(self,bot_loc,path,img_shortest_path,bot_localizer,frame_disp):
        Doing_pt = 0
        Done_pt = 0

        path_i = self.path_iter
        
        # Circle to represent car current location
        img_shortest_path = cv2.circle(img_shortest_path, bot_loc, 3, (0,0,255))

        if ( (type(path)!=int) and ( path_i!=(len(path)-1) ) ):
            curr_goal = path[path_i]
            # Mini Goal Completed
            if path_i!=0:
                img_shortest_path = cv2.circle(img_shortest_path, path[path_i-1], 3, (0,255,0),2)
                Done_pt = path[path_i-1]
            # Mini Goal Completing   
            img_shortest_path = cv2.circle(img_shortest_path, curr_goal, 3, (0,140,255),2)
            Doing_pt = curr_goal
        else:
            # Only Display Final Goal completed
            img_shortest_path = cv2.circle(img_shortest_path, path[path_i], 10, (0,255,0))
            Done_pt = path[path_i]

        if Doing_pt!=0:
            Doing_pt = self.bck_to_orig(Doing_pt, bot_localizer.transform_arr, bot_localizer.rot_mat)
            frame_disp = cv2.circle(frame_disp, (int(Doing_pt[0]),int(Doing_pt[1])), 3, (0,140,255),2)   
            #loc_car_ = self.bck_to_orig(loc_car, bot_localizer_obj.transform_arr, bot_localizer_obj.rot_mat_rev)
            #frame_disp = cv2.circle(frame_disp, (int(loc_car_[0]),int(loc_car_[1])), 3, (0,0,255))
         
            
        if Done_pt!=0:
            Done_pt = self.bck_to_orig(Done_pt, bot_localizer.transform_arr, bot_localizer.rot_mat_rev)
            if ( (type(path)!=int) and ( path_i!=(len(path)-1) ) ):
                pass
                #frame_disp = cv2.circle(frame_disp, (int(Done_pt[0]),int(Done_pt[1])) , 3, (0,255,0),2)   
            else:
                frame_disp = cv2.circle(frame_disp, (int(Done_pt[0]),int(Done_pt[1])) , 10, (0,255,0))  

        st = "len(path) = ( {} ) , path_iter = ( {} )".format(len(path),self.path_iter)        
        
        frame_disp = cv2.putText(frame_disp, st, (bot_localizer.orig_X+50,bot_localizer.orig_Y-30), cv2.FONT_HERSHEY_PLAIN, 1.2, (0,0,255))
        if config.debug and config.debug_motionplanning:
            cv2.imshow("maze (Shortest Path + Car Loc)",img_shortest_path)
        else:
            try:
                cv2.destroyWindow("maze (Shortest Path + Car Loc)")
            except:
                pass

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    

    def go_to_goal(self,bot_loc,path,velocity,velocity_publisher):



        angle_to_goal,distance_to_goal=self.angle_n_dist(bot_loc,(self.goal_pose_x,self.goal_pose_y))
        angle_to_turn=angle_to_goal-self.bot_angle_i

        speed=interp(distance_to_goal,[0,100],[0.2,1.5])
        angle=interp(angle_to_turn,[-360,360],[-4.0,4.0])


        if(distance_to_goal>=2):
            velocity.angular.z=angle

        if abs(angle<0.4):
            velocity.linear.x=speed
            
        elif(abs(angle<0.8)):
            velocity.linear.x=0.2
        else:
            velocity.linear.x=0.0

        if(self.goal_not_reached_flag or distance_to_goal<=1):
            velocity_publisher.publish(velocity)


        if(distance_to_goal<=8):
            velocity.linear.x=0.0
            velocity.angular.z=0.0

            if self.goal_not_reached_flag:
                velocity_publisher.publish(velocity)
                
            if self.path_iter==len(path)-1:
                self.goal_not_reached_flag=False

        else:
            self.path_iter+=1
            self.goal_pose_x=path[self.path_iter][0]
            self.goal_pose_y=path[self.path_iter][1]

            print(f"current_goal:{path[self.path_iter][0],path[self.path_iter][1]} ")

            
        




    def nav_path(self,bot_loc,path,velocity,velocity_publisher):

        if self.count>20:

            if not self.angle_rotation_computed:

                velocity.linear.x=0.0
                velocity_publisher.publish(velocity)
                self.bot_angle_i,_=self.angle_n_dist(self.init_loc,bot_loc)
                self.bot_angle_init=self.bot_angle_i
                self.bot_angle_rel=self.bot_angle_s-self.bot_angle_i
                self.angle_rotation_computed=True
            else:
                self.bot_angle_i=self.bot_angle_s-self.bot_angle_rel
                print("\n\nCar angle (Image From Relation) = {} I-S Relation {} Car angle (Simulation) = {}".format(self.bot_angle_i,self.bot_angle_rel,self.bot_angle_s))
                print("Car angle_Initial (Image) = ",self.bot_angle_init)
                print("Car loc {}".format(bot_loc))
                self.go_to_goal(bot_loc,path,velocity,velocity_publisher)
            
        else:
            if not self.pt_i_taken:
                self.init_loc=bot_loc
                self.pt_i_taken=True
            velocity.linear.x=1.0
            velocity_publisher.publish(velocity)
            self.count+=1




