%run detectors.py

#TODO: implement class, incapsulating input source
#possible inputs: video, camera, photo

CAMERA = 0
VIDEO  = 1
PHOTO  = 2

video_path = ""
video_file = ""

photo_path = "/Users/elijah/Dropbox/Programming/RoboCup/nao_cv/floor_desk/"
photo_file = "IMAGE_0_0_35.jpg"

output_path = "/Users/elijah/Dropbox/Programming/RoboCup/nao_cv/geometrical/chessboard_images/"

ROTATED_FRAME = True

def main ():
    INPUT_SOURCE = PHOTO

    cam_num = max (get_available_cameras ())

    cam = cv2.VideoCapture (cam_num)

    if (INPUT_SOURCE != CAMERA):
        cam.release ()

    if (INPUT_SOURCE == VIDEO):
        cam = cv2.VideoCapture (video_path + video_file)

    elif (INPUT_SOURCE == PHOTO):
        img = cv2.imread (photo_path + photo_file)

    #cv2.namedWindow ("frame", cv2.WINDOW_NORMAL)
    cv2.namedWindow ("frame")
    
    #cv2.resizeWindow ("frame", (640*2, 480*2))
    cv2.resizeWindow ("frame", (480, 640))

    str_num = 0

    calib_data = 0
    
    IMAGES_NUM = 10
    begin_timestamp = 0
    img_writing_mode = False
    written = 0
    
    x     = ""
    y     = "100"
    angle = ""

    unique_str = angle + "_" + x + "_" + y + "_"
    
    low_th  = (10, 10, 10)
    high_th = (220, 220, 220)

    detector = HSV_inRange_detector (low_th, high_th)

    while (True):
        if (INPUT_SOURCE == CAMERA or INPUT_SOURCE == VIDEO):
            ret, frame_ = cam.read ()

        elif (INPUT_SOURCE == PHOTO):
            frame_ = img.copy ()

        if (ROTATED_FRAME == False):
            frame = frame_
        
        else:
            sh = frame_.shape
            #frame = frame_.reshape ((sh [1], sh [0], sh [2]))
            frame = frame_.transpose ((1, 0, 2))
            frame = np.flip (frame, 1)
            #frame = frame [:] [ : : -1] [:]
        
        str_num = 0

        cv2.waitKey (1)    

        sz = 7
        shape = (sz, sz)
        
	
        
        cv2.imshow ("frame", frame)

        time.sleep (0.02)

        clear_output (wait=True)
        
        keyb = cv2.waitKey (1) & 0xFF
        
        if (keyb == ord('q')):
            break

    cam.release ()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main ()