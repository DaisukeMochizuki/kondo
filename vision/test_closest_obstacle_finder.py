import detectors
import cv2
import time
import os
#import matplotlib.pyplot as plt
 
#TODO: implement class, incapsulating input source
#possible inputs: video, camera, photo

CAMERA = 0
VIDEO  = 1
PHOTO  = 2

video_path = ""
video_file = ""

photo_path = "/Users/elijah/Dropbox/Programming/kondo/vision/images/"
photo_file = "obstacle.jpg"

output_path = "/Users/elijah/Dropbox/Programming/RoboCup/nao_cv/geometrical/chessboard_images/"

def main ():
    INPUT_SOURCE = PHOTO

    #cam_num = max (get_available_cameras ())

    #cam = cv2.VideoCapture (cam_num)

    #if (INPUT_SOURCE != CAMERA):
    #    cam.release ()

    #if (INPUT_SOURCE == VIDEO):
    #    cam = cv2.VideoCapture (video_path + video_file)

    if (INPUT_SOURCE == PHOTO):
        img = cv2.imread (photo_path + photo_file)

    detector = detectors.Detector ('closest_obstacle.txt')
    
    while (True):
        #if (INPUT_SOURCE == CAMERA or INPUT_SOURCE == VIDEO):
        #    ret, frame_ = cam.read ()

        if (INPUT_SOURCE == PHOTO):
            frame_ = img.copy ()

        #frame = cv2.cvtColor (frame_, cv2.COLOR_RGB2BGR)
        frame = frame_
                
        cv2.waitKey (1)    

        (obstacle_pixels, labels), _ = detector.detect (frame)

        #print ("keke")
        #print (labels)

        #draw obstacles on the frame
        result = frame.copy ()

        #os.system ("clear")
        #print (obstacle_pixels)

        #for pixel in obstacle_pixels:
        for i in range (len (obstacle_pixels)):
        #    #x, y, type = pixel

            x = i
            y = obstacle_pixels [i]

            type = labels [i]

            #if (y == 0):
            result = cv2.circle (result, (x, y), 5, (120 + type * 50, 150 + type * 150, 190 + type * 210), thickness = -1)

            #else:
            #    result = cv2.circle (result, (x, frame.shape [0] - y), 5,
            #        (120 + type * 50, 150 + type * 150, 190 + type * 210), thickness = -1)

        #stages = detector.get_stages ()
	
        #for i in range (2):
        #    cv2.imshow (str (i), stages [i])

        #processing_stages = detector.stages ()
	
	#resultant_frame = form_images (processing_stages)
        
        cv2.imshow ("frame", result)

        time.sleep (0.02)
       
        keyb = cv2.waitKey (1) & 0xFF
        
        if (keyb == ord('q')):
            break

    #cam.release ()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main ()