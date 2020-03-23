#!/usr/bin/python3

# import necessary modules
import cv2
import numpy as np


# function to mask the unwanted regions black
def focus_on_roi(img, vertices):
    mask = np.zeros_like(img)  # empty array
    cv2.fillPoly(mask, np.array([vertices], dtype=np.int32), 255)  # fill area outside with black
    result = cv2.bitwise_and(img, mask)  # combine with image
    return result


# to create coordinates for lines(lanes) given slope and intercept
def create_coordinates(img, line_parameters):
    slope, intercept = line_parameters
    y1 = img.shape[0]
    y2 = int(y1 * 0.65)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])


# for drawing lines
def display_lines(img, lns):
    line_image = np.zeros_like(img)  # empty array
    if lns is not None:
        for x1, y1, x2, y2 in lns:
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 8)  # draw line
    return line_image


# find slope and intercept of lanes detected for extending/contracting lanes to desired length
def average_slope_intercept(img, lns):
    left_fit = []
    right_fit = []
    for line in lns:
        x1, y1, x2, y2 = line.reshape(4)

        # It will fit the polynomial and the intercept and slope
        parameters = np.polyfit((x1, x2), (y1, y2), 1)  # find parameters of best fitting polynomial of degree one
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)
    left_line = create_coordinates(img, left_fit_average)
    right_line = create_coordinates(img, right_fit_average)
    return np.array([left_line, right_line])


if __name__ == "__main__":
    image = cv2.imread("./test_cases/image1.jpeg")  # read image
    height, width, channel_count = image.shape  # image parameters
    
	# set region of interest as a trapezium
    region_of_interest = [
            (0, height),
            (width/3, 2*height/3),
            (2*width/3, 2*height/3),
            (width, height)
        ]
	
	# conver to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # blur image to reduce noise so that we can focus on most significant lines
    blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
    # canny edge detection
    canny_image = cv2.Canny(blurred_image, 50, 150)
    # focus on our region of interest
    focused_image = focus_on_roi(canny_image, region_of_interest)
    # find lines by using probabilistic hough line transform
    found_lines = cv2.HoughLinesP(focused_image, 2, np.pi / 180, 100,
                                np.array([]), minLineLength=40,
                                maxLineGap=5)
                                
    averaged_lines = average_slope_intercept(image, found_lines)
	# image of lanes
    lines = display_lines(image, averaged_lines)
    # combine original image and lanes
    final_image = cv2.addWeighted(image, 0.8, lines, 1, 1)
	# save as new image
    cv2.imwrite("lanes_detected.jpeg", final_image)
    # display lanes detected
    cv2.imshow("Result", final_image)
    cv2.waitKey(0)
