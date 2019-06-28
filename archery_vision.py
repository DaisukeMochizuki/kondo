import cv2
import numpy as np
import time
import math
import matplotlib.pylab as plt
import random
from sklearn.metrics import mean_squared_error

r = 184
x0 = 642
y0 = 379


class Profiler(object):
    def __enter__(self):
        self._startTime = time.time()

    def __exit__(self):
        print("Elapsed time: {:.3f} sec".format(time.time() - self._startTime))


from timeit import default_timer as timer


def predict_new_position(current_position, time_shooting, angular_velocity, center):
    angular = angular_velocity * time_shooting
    x = math.fabs(current_position[0] - center[0]) * math.cos(angular) + math.fabs(
        current_position[1] - center[1]) * math.sin(angular)
    y = - math.fabs(current_position[0] - center[0]) * math.sin(angular) + math.fabs(
        current_position[1] - center[1]) * math.cos(angular)
    if current_position[0] < center[0]:
        x = - x
    if current_position[1] < center[1]:
        y = -y
    return (center[0] + x), (center[1] + y)


def predict_time(current_position, shooting_position, time_shooting, angular_velocity, center):
    pass


def determination_of_center(x1, y1, x2, y2, x3, y3):
    A = x2 - x1
    B = y2 - y1
    C = x3 - x1
    D = y3 - y1
    E = A * (x1 + x2) + B * (y1 + y2)
    F = C * (x1 + x3) + D * (y1 + y3)
    G = 2 * (A * (y3 - y2) - B * (x3 - x2))
    if G != 0:
        Cx = (D * E - B * F) / G
        Cy = (A * F - C * E) / G
        R = math.sqrt((Cx - x1) ** 2 + (Cy - y1) ** 2)
        return Cx, Cy, R


def scalar(x_1, y_1, x_2, y_2, x_3, y_3):
    x_2 = math.fabs(x_1 - x_2)
    x_3 = math.fabs(x_1 - x_3)
    y_2 = math.fabs(y_1 - y_2)
    y_3 = math.fabs(y_1 - y_3)
    sm = (x_2 * x_3 + y_2 * y_3)
    f = (math.sqrt(x_2 ** 2 + y_2 ** 2) * math.sqrt(x_3 ** 2 + y_3 ** 2))
    return math.acos(sm / f)


def super_img_position_mask(x, y):
    new = np.zeros(y.shape)
    for i in range(y.shape[2]):
        new[:, :, i] = x * y[:, :, i]
    return new


def mask(a, x1, x2, y1, y2, z1, z2):
    h_min = np.array((x1, y1, z1), np.uint8)
    h_max = np.array((x2, y2, z2), np.uint8)
    return cv2.inRange(a, h_min, h_max)


def find_circles_on_img(source_img, counter, final_array):
    hsv = cv2.cvtColor(source_img, cv2.COLOR_BGR2HSV)
    thresh = mask(hsv, 0, 238, 166, 255, 157, 255)
    new = super_img_position_mask(thresh, source_img)
    # print(new[0][0])
    new = new.astype(np.uint8)
    # print(new[0][0])
    new = cv2.medianBlur(new, 5)
    new = cv2.cvtColor(new, cv2.COLOR_BGR2GRAY)
    # cv2.imwrite('5.png',new)
    new = cv2.Canny(new, 350, 360)
    # cv2.imwrite('6.png',new)
    # hsv = cv2.cvtColor(new, cv2.COLOR_HSV2BGR )
    # hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(new, cv2.HOUGH_GRADIENT, 1.15, 0.0001, param1=85, param2=100, minRadius=5, maxRadius=300)
    print(circles.shape)
    try:
        # circles = np.uint16(np.around(circles))
        circles = circles.reshape(circles[0].shape)
        circles_mean = np.mean(circles, axis=0)
        circles_mean = np.uint16(np.around(circles_mean))
        # for i in circles[0,:]:
        # нарисовать центры окружностей
        # cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
        # cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
        cv2.circle(source_img, (circles_mean[0], circles_mean[1]), 2, (0, 0, 255), 3)
        # cv2.imshow('result', new)
        final_array[counter, [0, 1, 2]] = circles_mean
        # cv2.imwrite("videos/new" + str(counter) + ".jpg", img)
    except Exception as e:
        print(e)


def mse(final_array, source_circle):
    dist = []
    for el in final_array:
        tmp = ((el[0] - source_circle[0]) ** 2 + (el[1] - source_circle[1]) ** 2) ** 0.5
        dist.append(np.abs(tmp - source_circle[2]))
    return np.mean(dist)


def get_center_trajectory(frame_num=300):
    cap = cv2.VideoCapture(6)
    speed = np.zeros(shape=frame_num)
    trajectory = np.zeros(shape=(frame_num, 4))
    cap.read()
    for i in range(frame_num):
        # with Profiler() as p:
        flag, img = cap.read()
        timef = cap.get(cv2.CAP_PROP_POS_MSEC)
        if flag:
            find_circles_on_img(img, i, trajectory)
            # print(flag)
            # print(s)
        trajectory[i, 3] = timef
        print(timef)
        if i != 0:
            speed[i] = scalar(x0, y0, x_new, y_new, trajectory[i, 0], trajectory[i, 1]) / \
                       ((trajectory[i, 3] - trajectory[i - 1, 3]) / 1000)
        x_new = trajectory[i, 0]
        y_new = trajectory[i, 1]
    return trajectory, speed


    # getting circle from points
def trajectory_optimizer(trajectory):
    N = 50
    new = trajectory[:, :2]
    res = []
    circles = []
    for i in range(N):
        indexes = [random.randint(0, np.shape(trajectory)[0] - 1) for j in range(3)]
        points = new[indexes]

        circle = determination_of_center(points[0][0], points[0][1], points[1][0], points[1][1], points[2][0], points[2][1])
        try:
            circle[0]
        except:
            continue
        res.append(mse(trajectory, circle))
        circles.append(circle)
    return circles[int(np.argmin(res))]


traj, sp = get_center_trajectory()
circle = trajectory_optimizer(traj)

