import time
import math
import numpy as np
import cv2
import rospy
import PIL
import PIL.Image

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology



class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        # self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection for F1 car and rosbag
        self.sub_image = rospy.Subscriber('/D435I/color/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in normal rosbag 0011, 0056
        # self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in larger rosbag 830
        # self.sub_image = rospy.Subscriber('zed2/zed_node/rgb/image_rect_color', Image, self.img_callback, queue_size=1)

        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.pub_err = rospy.Publisher("lane_detection/error", Float32, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True


    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image, err = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)
        if err is not None:
            self.pub_err.publish(err)


    def gradient_thresh(self, img, thresh_min=25, thresh_max=100):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        #1. Convert the image to gray scale
        #2. Gaussian blur the image
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        #4. Use cv2.addWeighted() to combine the results
        #5. Convert each pixel to unint8, then apply threshold to get binary image


        ## TODO 
        #1. Convert the image to gray scale
        grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #2. Gaussian blur the image
        blur_gray = cv2.GaussianBlur(grayscale, (5, 5), 0)
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        sobelx = cv2.Sobel(blur_gray, cv2.CV_64F, 1, 0)  # x derivative
        sobely = cv2.Sobel(blur_gray, cv2.CV_64F, 0, 1)  # y derivative
        #4. Use cv2.addWeighted() to combine the results
        combine = cv2.addWeighted(sobelx, 0.5, sobely, 0.5, 0) # uncomment for GEM
        # combine = cv2.addWeighted(sobelx, 0.7, sobely, 0.3, 0) # uncomment for bag 0011, 0056
        # combine = cv2.addWeighted(sobelx, 0.9, sobely, 0.1, 0) # uncomment for bag 0830

        #5. Convert each pixel to uint8, then apply threshold to get binary image
        abs_sobel = np.absolute(combine)
        scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel)) 
        binary_output = np.where((scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max), 1, 0)

        ####

        return binary_output


    def color_thresh(self, img, thresh=(100, 255)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image
        #Hint: threshold on H to remove green grass
        ## TODO

        #1. Convert the image from RGB to HSL
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)   
        #2. Apply threshold on S channel to get binary image     
        s_channel = hls[:, :, 2]  # get saturation 
        l_channel = hls[:, :, 1]  # get lightness
        h_channel = hls[:, :, 0]  # get hue
        binary_output = np.where((h_channel < 40) & ((s_channel > thresh[0])), 1, 0) # uncomment for GEM, bag 0011, 0056
        # binary_output = np.where(((h_channel < 35) & (s_channel > 50)) | (l_channel > 150), 1, 0) # uncomment for bag 0830
        
        ####

        return binary_output


    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        ## TODO
        #1. Apply sobel filter and color filter on input image
        sobel = self.gradient_thresh(img)
        color = self.color_thresh(img)

        ####

        binaryImage = np.zeros_like(sobel)
        # binaryImage[(color==1)|(sobel==1)] = 1  # uncomment for bag 0011, 0056, 0830
        binaryImage[(sobel==1)] = 1   # uncomment for GEM
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)

        return binaryImage


    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        #3. Generate warped image in bird view using cv2.warpPerspective()

        ## TODO
        #1. Visually determine 4 source points and 4 destination points

        # int_list = img.astype(np.uint8)
        # im = PIL.Image.fromarray(int_list * 255)
        # im = im.convert("L")
        # im.save("pre_warp.jpeg")

        src = np.float32([[300, 400], [600, 400], [633, 479], [200, 479]]) # uncomment for GEM
        dest = np.float32([[100, 0], [400, 0], [400, 550], [100, 550]])   # uncomment for GEM
        # src = np.float32([[470, 250], [730, 250], [800, 360], [230, 360]])  # uncomment for bag 0011, 0056
        # dest = np.float32([[0, 0], [639, 0], [639, 479], [0, 479]])         # uncomment for bag 0011, 0056
        # src = np.float32([[500, 410], [690, 380], [1150, 719], [130, 719]]) # uncomment for bag 0830
        # dest = np.float32([[50, 0], [589, 0], [589, 479], [50, 479]])   # uncomment for bag 0830

        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        M = cv2.getPerspectiveTransform(src, dest)
        Minv = np.linalg.inv(M)

        #3. Generate warped image in bird view using cv2.warpPerspective()
        warped_img = cv2.warpPerspective(np.uint8(img), M, (640, 480))

        # im = PIL.Image.fromarray(warped_img * 255)
        # im = im.convert("L")
        # im.save("post_warp.jpeg")
        # breakpoint()
 
        return warped_img, M, Minv


    def detection(self, img):

        binary_img = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)
        err = 0
        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            left_fit = ret['left_fit']
            right_fit = ret['right_fit']
            nonzerox = ret['nonzerox']
            nonzeroy = ret['nonzeroy']
            left_lane_inds = ret['left_lane_inds']
            right_lane_inds = ret['right_lane_inds']
            err = ret['err']

        else:
            # Fit lane with previous result
            if not self.detected:
                ret = line_fit(img_birdeye)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']
                    err = ret['err']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                    self.detected = True

            else:
                left_fit = self.left_line.get_fit()
                right_fit = self.right_line.get_fit()
                ret = tune_fit(img_birdeye, left_fit, right_fit)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']
                    err = ret['err']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                else:
                    self.detected = False

            # Annotate original image
            bird_fit_img = None
            combine_fit_img = None
            if ret is not None:
                bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
                combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
            else:
                print("Unable to detect lanes")

            return combine_fit_img, bird_fit_img, err


if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)