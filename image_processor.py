#Basic image processing methods

def fill_holes (img):
    (h, w) = img.shape
    
    before_area = img.sum ()
    
    img_enlarged = np.zeros ((h + 2, w + 2), np.uint8)
    img_enlarged [1:h+1, 1:w+1] = img

    img_enl_not = cv2.bitwise_not (img_enlarged)
    th, im_th = cv2.threshold (img_enl_not, 220, 255, cv2.THRESH_BINARY_INV);

    im_floodfill = im_th.copy()

    h, w = im_th.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)

    cv2.floodFill(im_floodfill, mask, (0,0), 255);
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    im_out = im_th | im_floodfill_inv
    
    result = im_out [1:h-1, 1:w-1]
    
    return result