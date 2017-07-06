#import necessary packages 
import cv2
import numpy as numpy
import argparse

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True,
help = "path to the image dataset")
args = vars(ap.parse_args())

#load the image and convert to grayscale , blur
image = cv2.imread(args["image"])
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5,5), 0)
#edge detection 
canny = cv2.Canny(gray,40,100)
dilation = cv2.dilate(canny,None,iterations = 1)
cv2.imshow("Edge detected image",dilation)
cv2.waitKey(0)
