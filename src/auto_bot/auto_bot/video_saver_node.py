import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os




class VideoSubscriber(Node):

    def __init__(self):
        super().__init__('video_subscriber')
        self.image_sub= self.create_subscription(
            Image,
            '/upper_camera/image_raw',
            self.process_data,
            10)
        vid_path=os.path.join(os.getcwd(),"output_vid.avi")
        self.out=cv2.VideoWriter(vid_path,cv2.VideoWriter_fourcc('M','J','P','G'),30,(1280,720))
        self.Bridge=CvBridge()
        self.image_sub  # prevent unused variable warning

    def process_data(self, data):
            frame=self.Bridge.imgmsg_to_cv2(data,'bgr8')
            self.out.write(frame)
            cv2.imshow("Output",frame)
            cv2.waitKey(1)
            
        


def main(args=None):
    rclpy.init(args=args)

    video_subscriber = VideoSubscriber()

    rclpy.spin(video_subscriber)

    rclpy.shutdown()


if __name__ == '__main__':
    main()