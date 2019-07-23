#Detector incapsulates the whole detection process, which practically means image processing
#to certain stage and consequent extraction of the required object

#Image processor is a separate substance to process an image with a sequence of filters,
#such as to_gray, binarization, leave_biggest_connected_component, etc.

#Filter is an img-to-img transformation; generally from any shape to any shape

class Filter:
    def __init__(self):
	pass
    
    def apply (self, img):
	pass

class id:
    def __init__(self):
	pass
    
    def apply (self, img):
	return img

class to_grayscale: public Filter
    def __init__ (self):
	pass

    def apply (self, img):
	return cv2.cvtColor (img, cv2.COLOR_BGR2GRAY)

class inrange

class leave_biggest_connected_component

class filter_connected_components



class Image_processor:
    stages = {}

    filters = {}

    def __init__(self):
	pass

    def add_filter (new_filter, filter_name):
	filters.update ()

#Any detector (color-based, NN, template-based) is supposed to
#be a derivative class from Detector.

class Detector:
    #Basic class for detectors
    #Functions are to be reloaded
 
    def __init__(self):
        pass
    
    def detect(self, image):
        return ((0, 0), (0, 0))

#cv2.inRange for HSV color space mask obtainment
#Biggest connected component is considered as the needle

#class HSV_inRange_detector
    #...