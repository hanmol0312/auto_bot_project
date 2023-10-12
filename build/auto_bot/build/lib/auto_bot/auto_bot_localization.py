import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from .utilities import ret_smallest_obj,ret_largest_obj
from . import config

class auto_bot_localizer():
    def __init__(self):
        self.is_bg_extracted=False

        self.bg_model=[]
        self.loc_car=0
        self.maze_og=[]
        self.orig_X = 0
        self.orig_Y = 0
        self.orig_rows = 0
        self.orig_cols = 0
        self.transform_arr = []

        self.orig_rot = 0
        self.rot_mat = 0
        self.rot_mat_rev=0
    def update_frameofrefrence_parameters(self,X,Y,W,H,rot_angle):
        self.orig_X = X; self.orig_Y = Y; self.orig_rows = H; self.orig_cols = W; self.orig_rot = rot_angle # 90 degree counterClockwise
        self.transform_arr = [X,Y,W,H]
        # Rotation Matrix
        self.rot_mat = np.array(
                                [
                                 [ np.cos(np.deg2rad(self.orig_rot)) , np.sin(np.deg2rad(self.orig_rot))],
                                 [-np.sin(np.deg2rad(self.orig_rot)) , np.cos(np.deg2rad(self.orig_rot))]
                                ]
                               )
        self.rot_mat_rev = np.array(
                                [
                                 [ np.cos(np.deg2rad(-self.orig_rot)) , np.sin(np.deg2rad(-self.orig_rot))],
                                 [-np.sin(np.deg2rad(-self.orig_rot)) , np.cos(np.deg2rad(-self.orig_rot))]
                                ]
                               )
    
    @staticmethod
    def ret_rois_boundinghull(rois_mask,cnts):
        maze_enclosure = np.zeros_like(rois_mask)
        if cnts:
            cnts_ = np.concatenate(cnts)
            cnts_ = np.array(cnts_)
            cv2.fillConvexPoly(maze_enclosure, cnts_, 255)
        cnts_largest = cv2.findContours(maze_enclosure, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]# OpenCV 4.2
        hull = cv2.convexHull(cnts_largest[0])
        cv2.drawContours(maze_enclosure, [hull], 0, 255)
        return hull

    @staticmethod
    def get_centroid(cnt):
        m=cv2.moments(cnt)
        cx=int(m['m10']/m['m00'])
        cy=int(m['m01']/m['m00'])
        return (cy,cx)


    def get_car_loc(self,car_cnt,car_mask):

        bot_cnt=self.get_centroid(car_cnt)
        bot_cnt_arr=np.array([bot_cnt[1],bot_cnt[0]])
        bot_cnt_translated=np.zeros_like(bot_cnt_arr)

        bot_cnt_translated[0]=bot_cnt_arr[0]-self.orig_X
        bot_cnt_translated[1]=bot_cnt_arr[1]-self.orig_Y

        bot_on_maze=(self.rot_mat @ bot_cnt_translated.T).T
        rot_cols = self.orig_rows
        rot_rows = self.orig_cols
        bot_on_maze[0] = bot_on_maze[0] + (rot_cols * (bot_on_maze[0]<0) )  
        bot_on_maze[1] = bot_on_maze[1] + (rot_rows * (bot_on_maze[1]<0) )
        # Update the placeholder for relative location of car

        self.loc_car = (int(bot_on_maze[0]),int(bot_on_maze[1]))



    def bg_extract(self,frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges=cv2.Canny(gray,50,150,None,3)
        conts=cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[0]
        rois_mask=np.zeros((frame.shape[0],frame.shape[1]),dtype=np.uint8)

        for idx,_ in enumerate(conts):
            cv2.drawContours(rois_mask,conts,idx,255,-1)
        
        min_idx=ret_smallest_obj(conts) 
        rois_nocar_mask=rois_mask.copy()
        if min_idx!=-1:

            cv2.drawContours(rois_nocar_mask,conts,min_idx,0,-1)
            car_mask=np.zeros_like(rois_mask)
            cv2.drawContours(car_mask,conts,min_idx,255,-1)
            cv2.drawContours(car_mask,conts,min_idx,255,3)
            nocar_mask=cv2.bitwise_not(car_mask)
            frame_rm_car=cv2.bitwise_and(frame,frame,mask=nocar_mask)
            base_clr=frame_rm_car[0][0]
            ground_replicas=np.ones_like(frame)*base_clr

            self.bg_model=cv2.bitwise_and(ground_replicas,ground_replicas,mask=car_mask)
            self.bg_model=cv2.bitwise_or(self.bg_model,frame_rm_car)   

        hull=self.ret_rois_boundinghull(rois_mask,conts)
        [X,Y,W,H]=cv2.boundingRect(hull)
        maze=rois_nocar_mask[Y:Y+H,X:X+W]
        maze_occupencygrid=cv2.bitwise_not(maze)
        self.maze_og=cv2.rotate(maze_occupencygrid,cv2.ROTATE_90_COUNTERCLOCKWISE)

        self.update_frameofrefrence_parameters(X,Y,W,H,90)

        if (config.debug and config.debug_localization):
            cv2.imshow("1a. rois_mask",rois_mask)
            cv2.imshow("1b. frame_car_remvd",frame_rm_car)
            cv2.imshow("1c. Ground_replica",ground_replicas)
            cv2.imshow("1d. bg_model",self.bg_model)
            cv2.imshow("2. maze_og",self.maze_og)


    def localize_bot(self,curr_frame,disp_frame):

        if not self.is_bg_extracted:
            self.bg_extract(curr_frame)
            self.is_bg_extracted=True
            # foreground substraction

        change=cv2.absdiff(curr_frame,self.bg_model)
        change_gray=cv2.cvtColor(change,cv2.COLOR_BGR2GRAY)
        change_mask=cv2.threshold(change_gray,15,255,cv2.THRESH_BINARY)[1]
        car_mask,car_cnt=ret_largest_obj(change_mask)


        self.get_car_loc(car_cnt,car_mask)
        center, radii = cv2.minEnclosingCircle(car_cnt)
        car_circular_mask = cv2.circle(car_mask.copy(), (int(center[0]), int(center[1])), int(radii+(radii*0.4)), 255, 3)
        car_circular_mask = cv2.bitwise_xor(car_circular_mask, car_mask)
        disp_frame[car_mask>0]  = disp_frame[car_mask>0] + (0,64,0)
        disp_frame[car_circular_mask>0]  = (0,0,255)

        if (config.debug and config.debug_localization):
            cv2.imshow("1d. bg_model",self.bg_model)
            cv2.imshow("2. maze_og",self.maze_og)
            
            cv2.imshow("car_localized", disp_frame)
            cv2.imshow("change_mask(Noise Visible)", change_mask) 
            cv2.imshow("Detected_foreground(car)", car_mask)

        else:
            try:
                cv2.destroyWindow("1d. bg_model")
                cv2.destroyWindow("2. maze_og")
                
                cv2.destroyWindow("car_localized")
                cv2.destroyWindow("change_mask(Noise Visible)")
                cv2.destroyWindow("Detected_foreground(car)")

                cv2.destroyWindow("1a. rois_mask")
                cv2.destroyWindow("1b. frame_car_remvd")
                cv2.destroyWindow("1c. Ground_replica")

            except:
                pass


            
            



