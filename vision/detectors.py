import image_processing
import cv2
import json

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

