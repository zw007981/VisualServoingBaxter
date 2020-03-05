#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import sys
import tf
from sensor_msgs.msg import Image
import baxter_interface
from std_msgs.msg import Float32

rospy.init_node('trackleft', anonymous=True)

class tracking:
    def __init__(self):
        self.saveFile = open('record.txt', 'w')
        self.saveFile1 = open('record1.txt', 'w')
        self.limb_interface = baxter_interface.Limb("left")
        # start positions
        self.x = 0.62                        # x     = front back
        self.y = 0.14                        # y     = left right
        self.z = 0.14                        # z     = up down
        self.roll = -1.0 * math.pi              # roll  = horizontal
        self.pitch = 0.0 * math.pi               # pitch = vertical
        self.yaw = 0.0 * math.pi               # yaw   = rotation
        self.pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
        print(self.pose)
        # Enable the actuators
        baxter_interface.RobotEnable().enable()
        # set speed as a ratio of maximum speed
        self.limb_interface.set_joint_position_speed(0.5)
        # height of table
        self.table = -0.077
        # camera parameter
        self.cam_calib = 0.0025                     # meters per pixel at 1 meter
        self.width = 960                        # Camera resolution
        self.height = 600
        # bridge to use opencv to process images
        self.bridge = CvBridge()
        # subscribe image message from webcam
        self.image_sub = rospy.Subscriber(
            "/cameras/left_hand_camera/image", Image, self.callback)
        # setup topic for images with detection results
        self.image_pub = rospy.Publisher(
            '/image_bridge', Image, latch=True, queue_size=10)
        self.XL_coo = rospy.Publisher(
            '/Xcoordinate_left', Float32, latch=True, queue_size=10)
        self.YL_coo = rospy.Publisher(
            '/Ycoordinate_left', Float32, latch=True, queue_size=10)

    def callback(self, data):
        # convert images from ROS msg format to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # construct the argument parse and parse the arguments
        greenLower = (29, 65, 47)
        greenUpper = (84, 175, 255)
        #pts = deque(maxlen=args["buffer"])
        cv_image = cv2.medianBlur(cv_image, 5)
        cv_image = cv2.bilateralFilter(cv_image, 5, 100, 100)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            cv2.imshow("Image window1",  mask)
            cv2.circle(cv_image, (int(x), int(y)),
                       int(radius), (0, 255, 255), 2)
            cv2.circle(cv_image, (int(x), int(y)), 5, (0, 0, 255), -1)
            cv2.imshow("Image window2",  cv_image)
            cv2.waitKey(3)
            pixel_dx = y - (self.height / 2)
            pixel_dy = x - (self.width / 2)
            left = baxter_interface.Limb('left').endpoint_pose()
            e = tf.transformations.euler_from_quaternion(left['orientation'])
            zw1 = list(left['position'])
            x_offset = pixel_dx * self.cam_calib * (self.pose[2] - self.table)
            #print("x_offset", x_offset)
            y_offset = pixel_dy * self.cam_calib * (self.pose[2] - self.table)
            #print("y_offset", y_offset)
            x_real = x_offset + zw1[0]
            y_real = y_offset + zw1[1]
            x_real = round(x_real, 2)
            y_real = round(y_real, 2)
            self.XL_coo.publish(float(x_real))
            self.YL_coo.publish(float(y_real))
            self.saveFile.write(str(x_real))
            # self.saveFile.write('xcoordinate ')
            self.saveFile.write('\n')
            self.saveFile.write(str(y_real))
            # self.saveFile.write('ycoordinate\n')
            self.saveFile.write('\n')
            self.saveFile.flush()
	    self.saveFile1.write(str(round(zw1[0], 3)))
	    self.saveFile1.write('\n')
	    self.saveFile1.write(str(round(zw1[1], 3)))
	    self.saveFile1.write('\n')
	    self.saveFile1.flush()

def main(args):

    ic = tracking()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
