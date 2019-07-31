import image_processing
import cv2

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
    
    def add_filter (self, new_filter, filter_name):
        self.filters.append ((new_filter, filter_name))
    
    def get_stages (self):
        return self.stages

    def detect(self, image):
        self.stages.append (image)
	
        for filter, name in self.filters:
            curr_state = filter.apply (self.stages [-1].copy ())
            self.stages.append (curr_state)

        return self.stages [-1]

