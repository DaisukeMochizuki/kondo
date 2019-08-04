#!/usr/bin/env python
with_ros = False
import image_processing
import cv2
import json

if with_ros:
    import rospy
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import String
    from geometry_msgs.msg import Point
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
class filter_connected_components (Filter):
    def __init__ (self):
        pass

    def apply (self, img):
        tl, br = img

        x = int ((tl [0] + br [0]) / 2)
        y = br [1]

        return (x, y)


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
            self._sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback, queue_size=1)
            self.features_pub = rospy.Publisher('detector/features', Point, queue_size=1)
            #self.resulted_img = rospy.Publisher('detector/resulted_img', CompressedImage, queue_size=1)
	
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
        self.stages.append (image)
	
        for filter, name in self.filters:
            curr_state = filter.apply (self.stages [-1])
            self.stages.append (curr_state)

        return self.stages [-1]
    """if with_ros:
	def callback(self, image_msg):
            str_num = 0
            try:
                frame = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
            except CvBridgeError as e:
                print(e)

            #top left, bottom right
            #bbox_tl, bbox_br = detector.detect (frame)
	    #draw bbox on the frame
            #result = cv2.rectangle (frame.copy (), bbox_tl, bbox_br, (255, 0, 0), 5)

            #bottom point coordinates
            x, y = detector.detect (frame)

            #draw circle on the frame
            result = cv2.circle (frame.copy (), (x, y), 5, (120, 150, 190), thickness = -1)

            cv2.waitKey(2)

            cv2.imshow ("frame", result)
            print (x, y)

            #img_msg = CompressedImage()
            #img_msg.header.stamp = rospy.Time.now()
            #img_msg.format = "jpeg"
            #img_msg.data = np.array(cv2.imencode('.jpg', frame_with_bbox)[1]).tostring()
            # Publish new image
            #self.resulted_img.publish(img_msg)

            features_msg = Point(float(x), float(y), float(0))
            self.features_pub.publish(features_msg)

            #stages = detector.get_stages ()

            #for i in range (2):
            #    cv2.imshow (str (i), stages[i])"""
	
if __name__ == "__main__":
	if with_ros:
	    rospy.init_node('detector')
	    conf_file = rospy.get_param('~conf_file')
	    detector = Detector(conf_file)
	    rospy.spin()

