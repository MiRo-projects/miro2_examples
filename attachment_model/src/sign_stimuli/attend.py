import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os

class Attend:
    def callback_caml(self, ros_image):

        self.callback_cam(ros_image, 0)

    def callback_camr(self, ros_image):

        self.callback_cam(ros_image, 1)

    def callback_cam(self, ros_image, imno):

        # silently (ish) handle corrupted JPEG frames
        try:
            # convert compressed ROS image to raw CV image
            self.image[imno] = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
        except CvBridgeError as e:
            # print(e)
            pass

    def __init__(self, show_images = False):
        self.old_frame = [None, None]
        self.old_gray = [None, None]
        self.p0 = [None, None]
        self.color = np.random.randint(0, 255, (100, 3))
        self.image = [None, None]
        self.image_w = 640
        self.image_h = 0
        self.show_images = show_images

        # ROS -> OpenCV converter
        self.image_converter = CvBridge()

        # robot name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # subscribe
        self.sub_caml = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed",
                                         CompressedImage, self.callback_caml, queue_size=1, tcp_nodelay=True)
        self.sub_camr = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed",
                                         CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)

    def activate(self):
        ip1 = ip2 = None
        v1 = v2 = 0

        if self.image[0] is not None:
            ip1, v1 = self.findMovement(self.image[0], 0)
        if self.image[1] is not None:
            ip2, v2 = self.findMovement(self.image[1], 1)            
        
        target = 0.0
        cv = 0.0 

        if ip2 is not None or np.abs(v2) > np.abs(v1):
            target = 0 if ip2 == 0 else 1
            cv = v2
        elif ip1 is not None or np.abs(v1) > np.abs(v2):
            target = -1 if ip1 == 0 else 0
            cv  = v1
        else:
            cv = 0.0
        
        self.image = [None, None]
        return target, cv

    def findMovement(self, im, imageno):
        # Parameters for ShiTomasi corner detection
        # self.kinvel = self.kinvel - 0.8*self.kinvel

        feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

        # Parameters for Lucas Kanade optical flow
        lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

        
        if self.old_frame[imageno] is None:
            # Take first frame and find corners in it
            self.old_frame[imageno] = im
            self.old_gray[imageno] = cv2.cvtColor(self.old_frame[imageno], cv2.COLOR_BGR2GRAY)
            self.p0[imageno] = cv2.goodFeaturesToTrack(self.old_gray[imageno], mask=None, **feature_params)

        # Create a mask image for drawing purposes
        mask = np.zeros_like(self.old_frame[imageno])
        
        # Read new frame
        frame = im
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.image_w = frame.shape[1]
        self.image_h = frame.shape[0]

        # Calculate Optical Flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray[imageno], frame_gray, self.p0[imageno], None, **lk_params
        )

        target = None
        vf =  0

        if p1 is None:
            self.p0[imageno] = cv2.goodFeaturesToTrack(self.old_gray[imageno], mask=None, **feature_params)
            p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray[imageno], frame_gray, self.p0[imageno], None, **lk_params
            )

        # Select good points
        good_new = p1[st == 1]
        good_old = self.p0[imageno][st == 1]

        result = np.array([0.0, 0.0])
        counter = np.array([0.0, 0.0])

        # Draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()

            v = np.sqrt((a - c)**2 + (b - d)**2)

            if v < 2:
                continue

            idx = int(c/(0.5*self.image_w))
            counter[idx if idx <= 1 else 1] += 1
            
            result += np.array([a-c, b-d])
            

        if np.linalg.norm(result) > 2:
            target = self.image_w/4.0 if counter[0]>counter[1] else 3.0*self.image_w/4.0
            mask = cv2.line(mask, (int(target), int(100)), (int(target) + int(result[0]), 100 + int(result[1])), self.color[i].tolist(), 2)
            frame = cv2.circle(frame, (int(target), int(100)), 20, self.color[i].tolist(), -1)
            
            target = 0 if counter[0]>counter[1] else self.image_w
            vf = result[0]
                
        
        self.p0[imageno] = good_new.reshape(-1, 1, 2)
        # Update the previous frame and previous points
        self.old_gray[imageno] = frame_gray.copy()

        if self.show_images:
            # Display the demo
            img = cv2.add(frame, mask)
            cv2.imshow(str(imageno), img)
            # cv2.imshow('frame', im)
            if(imageno == 1):
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return target, vf
        return target, vf

