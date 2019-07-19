#Basic image processing methods

#Method for filling holes in connected components of the mask
#In other words, brings all the connected components to simply connected

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

#Method for bringing 3-channel image to RG-chromaticity color space
#Essentially it is a kind of normalization such that the absolute
#values of the intensity are neglected; the components ratios are not

def to_RG_chromaticity (img):
    (h, w, d) = img.shape
    
    norm = np.zeros ((h, w), np.float)
    norm = img [:, :, 0].astype ('float') +\
           img [:, :, 1].astype ('float') +\
           img [:, :, 2].astype ('float')
    
    norm [norm == 0] = 5
    
    turned = np.zeros (img.shape, np.uint8)
    turned [:, :, 0] = ((img [:, :, 0].astype ('float')) / norm * 255).astype ('uint8')
    turned [:, :, 1] = ((img [:, :, 1].astype ('float')) / norm * 255).astype ('uint8')
    turned [:, :, 2] = ((img [:, :, 2].astype ('float')) / norm * 255).astype ('uint8')
    
    return turned

#Method for finding the bounding box of the connected component having the biggest value
#of the given criterion, i.e. height, width, area.
#Not implemented yet.

def find_bounding_box (mask):
    result = np.array (mask)
    output = cv2.connectedComponentsWithStats (mask, 8, cv2.CV_32S)
    labels_num = output      [0]
    labels     = output      [1]
    stats      = output      [2]
    sz         = stats.shape [0]
    
    max_w     = 0
    max_label = 0
    
    for label_num in range (1, sz - 1):
        if (stats [label_num, cv2.CC_STAT_AREA] > max_w):
            max_w = stats [label_num, cv2.CC_STAT_AREA]
            max_label = label_num
    
    top    = stats [max_label, cv2.CC_STAT_TOP]
    left   = stats [max_label, cv2.CC_STAT_LEFT]
    width  = stats [max_label, cv2.CC_STAT_WIDTH]
    height = stats [max_label, cv2.CC_STAT_HEIGHT]
    
    return (left, top), (left + width, top + height)
