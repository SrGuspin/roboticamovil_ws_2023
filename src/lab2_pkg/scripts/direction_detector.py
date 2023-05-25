#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
import cv2 as cv
import numpy as np


def detectar(imagen):

    # img = cv.imread(cv.samples.findFile('arrow_right.png'))
    img = cv.imread(cv.samples.findFile(imagen))
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(gray, 50, 150, apertureSize=3)
    lines = cv.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=5, maxLineGap=190)
    sumax = 0
    sumay = 0
    for line in lines:
        x1, y1, x2, y2 = line[0]
        sumax += x1 + x2
        sumay += y1 + y2

    if sumax > 12000:
        print("right")
    else:
        print("left")


if __name__ == '__main__':

    if len(str(sys.argv)) > 1:
        lectura = sys.argv[1]
        detectar(lectura)

    else:
        print("Escribiste mal el comando para ejecutar este código")
