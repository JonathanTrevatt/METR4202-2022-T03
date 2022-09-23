import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors
from matplotlib.colors import hsv_to_rgb
import numpy as np


def main():

    image_dir = "./images/"
    image_name = "sample.png"
    image = cv2.imread(image_dir + image_name)  # Read image

    plt.imshow(image)
    plt.show()

    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    # corners: A list containing the (x, y)-coordinates of our detected ArUco markers
    # ids: The ArUco IDs of the detected markers
    # rejected: A list of potential markers that were found but ultimately rejected due to the inner code of the
    # marker being unable to be parsed (visualizing the rejected markers is often useful for debugging purposes)

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
