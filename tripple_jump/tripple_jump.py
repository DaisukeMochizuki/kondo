from kondo import Kondo
import sys
import cv2
import numpy as np

sys.path.append("kondo")

alfa_param = 0.1  # radian
line_param = 10
dist_param = 50


def calc_dist(b):
    diff = []
    for i in range(len(b) - 1):
        diff.append(b[i + 1] - b[i])
    diff.sort()
    return diff


def draw_lines(img, mask=None):
    # img = cv2.imread('src.png')
    # gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    gray = img
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    low_threshold = 50
    high_threshold = 150
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

    rho = 1  # distance resolution in pixels of the Hough grid
    # rho = 2  # distance resolution in pixels of the Hough grid

    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    # theta = np.pi / 80  # angular resolution in radians of the Hough grid

    # threshold = 15  # minimum number of votes (intersections in Hough grid cell)
    # threshold = 25  # minimum number of votes (intersections in Hough grid cell)
    # threshold = 35  # minimum number of votes (intersections in Hough grid cell)
    threshold = 45  # minimum number of votes (intersections in Hough grid cell)

    # min_line_length = 50  # minimum number of pixels making up a line
    min_line_length = 100  # minimum number of pixels making up a line

    # max_line_gap = 20  # maximum gap in pixels between connectable line segments
    # max_line_gap = 30  # maximum gap in pixels between connectable line segments
    # max_line_gap = 40  # maximum gap in pixels between connectable line segments
    max_line_gap = 60  # maximum gap in pixels between connectable line segments

    line_image = np.copy(img) * 0  # creating a blank to draw lines on

    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                            min_line_length, max_line_gap)

    if lines is None:
        return img, np.zeros(img.shape, np.uint8), None

    print(len(lines))
    new_lines = []

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 > 400 or x1 < 100:
                continue
            if x2 > 400 or x2 < 100:
                continue
            if y1 < 150 or y2 < 150:
                continue
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)
            new_lines.append(line)
            # cv2.circle (line_image, (x1, y1), 6, (20, 30, 240), -1)
            # cv2.circle (line_image, (x2, y2), 6, (20, 30, 240), -1)
    # Draw the lines on the  image
    lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)
    # cv2.imshow (name, lines_edges)
    return lines_edges, line_image, new_lines


def num_of_lines(lines):
    b = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            b.append((y1 * x2 - y2 * x1) / (x2 - x1))
    num_of_lines = 0
    b.sort()
    diff = calc_dist(b)
    if diff > line_param:
        num_of_lines += 1
    return num_of_lines


kondo = Kondo()
kondo.walk(2)
kondo.tilt()
cap = cv2.VideoCapture(6)

# bad frame
cap.read()
# good one
ret, frame = cap.read()
lines_edges, line_image, new_lines = draw_lines(frame)
N = num_of_lines(new_lines)
print(N)
cv2.imwrite("frame.jpg", lines_edges)

# calc alfa
tan_alfa = []
for line in new_lines:
    for x1, y1, x2, y2 in line:
        tan_alfa.append(np.abs(x1 - x2) / np.abs(y1 - y2))
alfa = np.arctan(np.median(tan_alfa))
print("angle = ", alfa)
kondo.straighten()

if np.abs(alfa) > alfa_param:
    kondo.turn(alfa)

# calc dist
# y = new_lines[::]
# dist = max(y)
dist = 0

# num of lines
N = num_of_lines(new_lines)
if (N < 3) and (dist < dist_param):
    kondo.triple_jump()
else:
    print("fake tj")
    kondo.triple_jump()
