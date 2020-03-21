# import necessary libraries
import numpy as np
import cv2
import sys


# initialise values of colors
red = [0, 0, 255]
green = [0, 255, 0]
yellow = [0, 255, 255]

# read image
image = cv2.imread(sys.argv[1])
# based on a threshold value, convert image to binary, ie, consisting of two colors
ret, thresh = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
# convert pixel data to list data type for ease of handling
thresh = np.array(thresh).tolist()

print("")

# check for color
for line in thresh:
    if red in line:
        print(">>> COLOR : RED")
        quit(0)
    elif yellow in line:
        print(">>> COLOR : YELLOW")
        quit(0)
    elif green in line:
        print(">>> COLOR : GREEN")
        quit(0)
    else:
        pass

print(">>> ERROR : Unable to recognise color :(")
quit(-1)
