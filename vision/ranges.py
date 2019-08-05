import cv2
import numpy as np
from matplotlib import pyplot as plt
from detectors import inrange
import os
#from image_processing import to_three

def nothing (x):
    pass

cv2.namedWindow ('Colorbars')

cv2.createTrackbar ("l1", "Colorbars", 0, 255, nothing)
cv2.createTrackbar ("h1", "Colorbars", 0, 255, nothing)
cv2.createTrackbar ("l2", "Colorbars", 0, 255, nothing)
cv2.createTrackbar ("h2", "Colorbars", 0, 255, nothing)
cv2.createTrackbar ("l3", "Colorbars", 0, 255, nothing)
cv2.createTrackbar ("h3", "Colorbars", 0, 255, nothing)

img = cv2.imread('images/basket/3.jpg')
#img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)

low_th  = (57, 150, 110)
high_th = (67, 160, 120)

inrange_filter = inrange (low_th, high_th)

while(1):
    l1 = cv2.getTrackbarPos ("l1", "Colorbars")
    h1 = cv2.getTrackbarPos ("h1", "Colorbars")
    l2 = cv2.getTrackbarPos ("l2", "Colorbars")
    h2 = cv2.getTrackbarPos ("h2", "Colorbars")
    l3 = cv2.getTrackbarPos ("l3", "Colorbars")
    h3 = cv2.getTrackbarPos ("h3", "Colorbars")

    low_th  = (l1, l2, l3)
    high_th = (h1, h2, h3)

    inrange_filter.set_ths (low_th, high_th)

    mask = inrange_filter.apply (img)

    enlighted = img.copy ()
    enlighted [:, :, 2] = np.add (enlighted [:, :, 0], np.multiply (mask, 0.8))

    cv2.imshow ("enlighted", enlighted)

    os.system ('clear')    
    print (low_th, high_th)

    if (cv2.waitKey(1) & 0xFF == 27):
        break

cv2.destroyAllWindows()