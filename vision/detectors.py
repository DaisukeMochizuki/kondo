#!/usr/bin/env python
with_ros = True
import image_processing
import cv2
import json

if with_ros:
    import rospy
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import String
    from geometry_msgs.msg import Point, Polygon
    from cv_bridge import CvBridge, CvBridgeError
    import cv2
    import numpy as np

#Filter is an img-to-img transformation; generally from any shape to any shape

class Filter:
    def __init__(self, name_):
        self.name = name_
    
    def apply (self, img):
        return img

#class tograyscale (Filter):
#    def __init__ (self):
#        pass
#
#    def apply (self, img):
#        return cv2.cvtColor (img, cv2.COLOR_BGR2GRAY)

class inrange (Filter):
    def __init__ (self, low_th_, high_th_):
        self.low_th  = low_th_
        self.high_th = high_th_

    def apply (self, img):
        return cv2.inRange (img, self.low_th, self.high_th)

#find bbox of the connected component with maximal area
class max_area_cc_bbox (Filter):
    def __init__ (self):
        pass

    def apply (self, img):
        return image_processing.find_max_bounding_box (img)

#returns bottom point of the bbox, middle by x axis
class bottom_bbox_point (Filter):
    def __init__ (self):
        pass

    def apply (self, img):
        tl, br = img

        x = int ((tl [0] + br [0]) / 2)
        y = br [1]

        return (x, y)

#should simply incapsulate basic processing function
#class filter_connected_components

#------------------------------------------------------

#Detector incapsulates the whole detection process, which practically means image processing
#to certain stage and consequent extraction of the required object

#Any detector (color-based, NN, template-based) is supposed to
#be set as a sequence of filters. The idea is obviously taken from NNs

class Detector:
    filters = []
    
    #processing stages (for debugging purposes)
    stages  = []

    def __init__(self):
        pass

    def __init__(self, detector_filename):
        if with_ros:
            self._cv_bridge = CvBridge()
            self._sub = rospy.Subscriber('/camera/image_raw_rhoban', Image, self.basket_callback, queue_size=1)
            self.basket_top = rospy.Publisher('detectors/basket_top', Point, queue_size=1)
            self.basket_bottom = rospy.Publisher('detectors/basket_bottom', Point, queue_size=1)
            self.obstacles = rospy.Publisher('detectors/obstacles', Polygon, queue_size=1)


            self.resulted_img = rospy.Publisher('detectors/resulted_img', CompressedImage, queue_size=1)
	
        with open (detector_filename) as f:
            data = json.load(f)

        for filter in data ["filters"]:
            filter_name = filter ["name"]

            if (filter_name == "inrange"):
                low_th   = (int (filter ["l1"]), int (filter ["l2"]), int (filter ["l3"]))
                high_th  = (int (filter ["h1"]), int (filter ["h2"]), int (filter ["h3"]))

                #print (low_th)

                new_filter = inrange (low_th, high_th)

            if (filter_name == "max_area_cc_bbox"):
                new_filter = max_area_cc_bbox ()

            if (filter_name == "bottom_bbox_point"):
                new_filter = bottom_bbox_point ()

            self.add_filter (new_filter, filter ["name"])
    
    def add_filter (self, new_filter, filter_name):
        self.filters.append ((new_filter, filter_name))
    
    def get_stages (self):
        return self.stages

    def detect(self, image):
        self.stages = []
        self.stages.append (image)
	
        for filter, name in self.filters:
            curr_state = filter.apply (self.stages [-1])
            self.stages.append (curr_state)

        return self.stages [-1]


    if with_ros:

        def get_hsv_image(self, ros_image_msg):
            try:
                frame = self._cv_bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding="passthrough")
                frame = cv2.cvtColor(frame, cv2.COLOR_YCrCb2RGB)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                return frame        
            except CvBridgeError as e:
                print(e)

        def basket_callback(self, img_msg):
            str_num = 0
            frame = self.get_hsv_image(img_msg)
            #print(frame.shape)
	        #cv2.imshow ("frame", frame)
            #top left, bottom right
            bbox_tl, bbox_br = self.detect(frame)
            print(bbox_tl, bbox_br)
            #calc basket top and bottom
            x_b = (bbox_br[0] + bbox_tl[0])/2
            y_b = bbox_br[1]
            x_t = (bbox_br[0] + bbox_tl[0])/2
            y_t = bbox_tl[1]
	    #draw bbox on the frame
            #result = cv2.rectangle (frame.copy (), bbox_tl, bbox_br, (255, 0, 0), 5)
	        #frame = cv2.cvtColor(frame, cv2.COLOR_YCR_CB2HSV)
	        #frame  = cv2.cvtColor(frame, cv2.COLOR_YCrCb2RGB)
	        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #bottom point coordinates
            #x, y = detector.detect (frame)

            #draw circle on the frame
            #result = cv2.circle (frame.copy (), (int(x), int(y)), 5, (120, 150, 190), thickness = -1)

            cv2.waitKey(2)

            #cv2.imshow ("frame", result)
            #print (x, y)

           # img_msg = CompressedImage()
           # img_msg.header.stamp = rospy.Time.now()
            #img_msg.format = "jpeg"
          #  img_msg.data = np.array(cv2.imencode('.jpg', frame_with_bbox)[1]).tostring()
         #  # Publish new image
           # self.resulted_img.publish(img_msg)

            basketT_msg = Point(float(x_t), float(y_t), float(0))
            basketB_msg = Point(float(x_b), float(y_b), float(0))
            self.basket_top.publish(basketT_msg)
            self.basket_bottom.publish(basketB_msg)

            
            
            

            #stages = detector.get_stages ()

           # for i in range (2):
            #    cv2.imshow (str (i), stages[i])


        
        
        # obstacles detection and publishing

        #def obstacles_callback(self, img_msg):
            frame = self.get_hsv_image(img_msg)

            #detected_obstacles = self.detect(frame)
            detected_obstacles = tuple(Point(1.0, 2.0, 0.0), Point(14.0, 2.5, 2.0), Point(11.0, 142.0, 1.0))
            self.obstacles.publish(detected_obstacles)

	
if __name__ == "__main__":
	if with_ros:
	    rospy.init_node('detectors')
	    conf_file = rospy.get_param('~conf_file')
	    detector = Detector(conf_file)
	    rospy.spin()
