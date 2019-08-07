#!/usr/bin/env python
with_ros = False
import image_processing
import cv2
import json
import numpy as np

#TODO: Move parameters parsing into the filters constructors from Detector constructor
#TODO: Implement simultaneous stages displaying in single window
#TODO: Document the logics behind the project architecture, filters creation
#TODO: Refactor parameters extraction in find_obstacles_distances creation, automate
#      types number obtainment
#TODO: Move code to standard Python style
#TODO: Add morphological filters, blurring filter

#------------------------------------------------------------------------------------

#TODO_FUTURE: Make up a way to plug filters in another filters.
#             Closest obstacle finder uses inrange, morphology, connected components filtering,
#             iterating

#TODO_FUTURE: Filter can store its parameters in a dictionary

if with_ros:
    import rospy
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import String
    from geometry_msgs.msg import Point
    from cv_bridge import CvBridge, CvBridgeError
    import cv2
    import numpy as np

#Filter is an img-to-img transformation; generally from any shape to any shape
#Previous comment was written in the very beginning of the development
#Filter is an anything-to-anything transformation

class Filter:
    def __init__(self, name_):
        self.name = name_
        self.success = []
    
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
        Filter.__init__ (self, "inrange")

        #self.set_ths (low_th_, high_th_)
        self.low_th  = low_th_
        self.high_th = high_th_

    def set_ths (self, low_th_, high_th_):
        self.low_th  = low_th_
        self.high_th = high_th_

    def apply (self, img):
        
        return cv2.inRange (img, self.low_th, self.high_th)

#find bbox of the connected component with maximal area
class max_area_cc_bbox (Filter):
    def __init__ (self):
        Filter.__init__ (self, "max_area_cc_bbox")

    def apply (self, img):
        result, success_curr = image_processing.find_max_bounding_box (img)

        self.success.append (success_curr)
        return result

#returns bottom point of the bbox, middle by x axis
class bottom_bbox_point (Filter):
    def __init__ (self):
        Filter.__init__ (self, "bottob_bbox_point")

    def apply (self, img):
        tl, br = img

        x = int ((tl [0] + br [0]) / 2)
        y = br [1]

        return (x, y)

#should simply incapsulate basic processing function
class filter_connected_components (Filter):
    def __init__ (self):
        Filter.__init__ (self, "filter_connected_components")

    def apply (self, img, area_low = -1, area_high = -1, hei_low = -1, hei_high = -1,
               wid_low = -1, wid_high = -1, den_low = -1, den_high = -1):
        return image_processing.filter_connected_components (img, area_low, area_high,
               hei_low, hei_high, wid_low, wid_high, den_low, den_high)

#finds pixel distance (by y axis) from the bottom of the frame to the closest obstacle
#returns list of points (x, y, obstacle type)
class find_obstacles_distances (Filter):
    def __init__ (self, ranges_):
        Filter.__init__ (self, "find_obstacles_distances")
        self.ranges = ranges_
        self.inrange_filter = inrange ((0, 0, 0), (255, 255, 255))
        self.cc_filter = filter_connected_components ()

    #def get_obstacles(img):
    #    smart_gray = 0.5 * img[:,:,2] + 0.5 * img[:,:,1]
    #    converted = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    #
    #    # white color mask
    #    lower = np.uint8([100, 100, 100])
    #    upper = np.uint8([120, 200, 200])
    #    binarized = cv2.inRange(converted, lower, upper)
    #
    #    op_ker = 12
    #    cl_ker = 12
    #    morph = binarized.astype('uint8')
    #    morph = cv2.morphologyEx(binarized, cv2.MORPH_OPEN, np.ones((op_ker, op_ker),np.uint8))
    #    morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, np.ones((cl_ker,cl_ker),np.uint8))
    #
    #    return morph

    def _get_obstacles_dists (self, obstacles):
        obstacles_flipped = cv2.flip (obstacles, 0)
        distances = np.argmax (obstacles_flipped, axis=0)

        #print ("fuck")
        #print (distances)

        return distances

    def apply (self, img):
        result = []
        labels = []

        sh = img.shape

        for i in range (sh [1]):
            labels.append (0)

        filled = False

        for range_num in range (len (self.ranges)):
            range_ = self.ranges [range_num]

            self.inrange_filter.set_ths (range_ [0], range_ [1])
            mask = self.inrange_filter.apply (img)
            mask = self.cc_filter.apply (mask, 10)

            cv2.imshow ("blyad", mask)

            op_ker = 12
            cl_ker = 12

            morph = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((op_ker, op_ker),np.uint8))
            morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, np.ones((cl_ker,cl_ker),np.uint8))

            temp_result = self._get_obstacles_dists (morph)

            if (filled == False):
                filled = True
                result = temp_result.copy ()

            for i in range (len (temp_result)):
                if (temp_result [i] != 0):
                    temp_result [i] = sh [0] - temp_result [i]

            for i in range (len (temp_result)):
                if (temp_result [i] <= result [i] and temp_result [i] != 0):
                    result [i] = temp_result [i]
                    labels [i] = range_num + 1

        #for i in range (sh [1]):
        #    result.append ((i, 200, i))

        #print ("ll")
        #print (labels)

        return result, labels

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
            data = json.load (f)

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

            if (filter_name == "find_obstacles_distances"):
                types_num = int (filter ["types_num"])
                
                ranges = []

                for i in range (types_num):
                    type_num = str (i + 1)

                    low_th   = (int (filter [type_num + "l1"]),
                                int (filter [type_num + "l2"]),
                                int (filter [type_num + "l3"]))

                    high_th  = (int (filter [type_num + "h1"]),
                                int (filter [type_num + "h2"]),
                                int (filter [type_num + "h3"]))

                    ranges.append ((low_th, high_th))
                
                new_filter = find_obstacles_distances (ranges)

            self.add_filter (new_filter, filter ["name"])
    
    def add_filter (self, new_filter, filter_name):
        self.filters.append ((new_filter, filter_name))
    
    def get_stages (self):
        return self.stages

    def detect(self, image):
        self.stages.append (image)
	
        success = True

        for filter, name in self.filters:
            curr_state = filter.apply (self.stages [-1])
            self.stages.append (curr_state)

            if (len (filter.success) != 0 and filter.success [-1] == False):
                success = False

        return self.stages [-1], success

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

