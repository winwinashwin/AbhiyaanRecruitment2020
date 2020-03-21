# necessary imports
import cv2
import numpy as np
import sys
import matplotlib.pyplot as plt


def focus_on_roi(img, vertices):
    mask = np.zeros_like(img)
    mask_color = 255  # mask unwanted region black
    cv2.fillPoly(mask, vertices, mask_color)
    result = cv2.bitwise_and(img, mask)
    return result


def draw_lines(img, lines):
    img = np.copy(img)
    blank = np.zeros(img.shape, dtype=np.uint8)
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(blank, (x1, y1), (x2, y2), (255, 0, 0), thickness=10)

    img = cv2.addWeighted(img, 1, blank, 1, 0.0)
    return img


if __name__ == "__main__":
    try:
        image = cv2.imread(sys.argv[1])
        # image = cv2.imread(".\\test_cases\\image1.jpeg")

    except:
        image = ""
        print("Error in reading image, try again.")
        quit(-1)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    height, width, channel_count = image.shape
    region_of_interest = [
        (0, height),
        (width/3, 2*height/3),
        (2*width/3, 2*height/3),
        (width, height)
    ]

    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    canny_image = cv2.Canny(gray_image, 100, 200)
    cropped_image = focus_on_roi(canny_image, np.array([region_of_interest], np.int32))

    found_lines = cv2.HoughLinesP(cropped_image,
                            rho=6,
                            theta=np.pi/60,
                            threshold=160,
                            lines=np.array([]),
                            minLineLength=40,
                            maxLineGap=25)

    image_with_lines = draw_lines(image, found_lines)

    cv2.imwrite("detected.jpeg", cv2.cvtColor(image_with_lines, cv2.COLOR_RGB2BGR))
    plt.imshow(image_with_lines)
    plt.show()
