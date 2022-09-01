# import the necessary packages
import argparse
import imutils
import cv2
import matplotlib.pyplot as plt
import sys
import numpy as np

# construct the command line argument parser and parse the arguments
# --image: The path to the input image containing any ArUco tags we want to detect
# --type: The type of ArUco tags that we’ll be detecting
# TODO - implement use as both commandline and as package
"""ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="path to input image containing ArUCo tag")
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to detect")
args = vars(ap.parse_args())"""

image_path = "./images/sample.png"
# Reading an image in default mode
image = cv2.imread(image_path)
type_arg = "DICT_4X4_50"
args = {"image": image_path, "type": type_arg}

if image is None:
    print("Check file path")

# define names of each possible ArUco tag OpenCV supports - ie converts human readable "type" to a unique ID
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

# load the input image from disk and resize it
print("[INFO] loading image...")
image = cv2.imread(args["image"])
image = imutils.resize(image, width=600)
# verify that the supplied ArUCo tag exists and is supported by OpenCV
if ARUCO_DICT.get(args["type"], None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
    sys.exit(0)
# load the ArUCo dictionary, grab the ArUCo parameters, and detect the markers
print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()


# Extract marker corners (which are always returned in order top-left, top-right, bottom-right, and bottom-left)
def extract_marker_corners(markerCorner):
    corners = markerCorner.reshape((4, 2))
    (topLeft, topRight, bottomRight, bottomLeft) = corners
    # convert each of the NumPy array (x, y)-coordinate pairs to integers
    topRight = (int(topRight[0]), int(topRight[1]))
    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    topLeft = (int(topLeft[0]), int(topLeft[1]))
    corners = [topRight, bottomRight, bottomLeft, topLeft]
    return corners


def draw_quad(image, topLeft, topRight, bottomRight, bottomLeft):
    cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
    cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
    cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
    cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
    return image

def draw_quad_centerpoint(image, topLeft, topRight, bottomRight, bottomLeft):
    # compute and draw the center (x, y)-coordinates of the ArUco marker
    (cX, cY) = get_quad_centerpoint(topLeft, topRight, bottomRight, bottomLeft)
    cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
    # draw the ArUco marker ID on the image
    return image

def draw_quad_markerID(image, topLeft, topRight, bottomRight, bottomLeft, markerID):
    # draw the ArUco marker ID on the image
    cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    print("[INFO] ArUco marker ID: {}".format(markerID))
    return image

def get_quad_centerpoint(topLeft, topRight, bottomRight, bottomLeft):
    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
    return cX, cY

def warp_quad_to_square(image, topLeft, topRight, bottomRight, bottomLeft):
    # Coordinates of quadrangle vertices in the source image.
    pts1 = np.float32([topLeft, topRight, bottomRight, bottomLeft])
    # Coordinates of the corresponding quadrangle vertices in the destination image.
    side_length = topRight[0] - topLeft[0]
    pts2 = np.float32(
        [topLeft,
         [topLeft[0] + side_length, topLeft[1]],  # New top right
         [topLeft[0] + side_length, topLeft[1] + side_length],  # New bottom right
         [topLeft[0], topLeft[1] + side_length]  # New bottom left
         ])
    # Apply Perspective Transform Algorithm
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    image = cv2.warpPerspective(image, matrix, (image.shape[0], image.shape[1]))
    return image

def process_image(image, arucoDict, arucoParams):
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    # Visualise ArUco markers
    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # Extract marker corners (which are always returned in order top-left, top-right, bottom-right, and bottom-left)
            (topLeft, topRight, bottomRight, bottomLeft) = extract_marker_corners(markerCorner)

            # draw the bounding box of the ArUCo detection on the image
            image = draw_quad(image, topLeft, topRight, bottomRight, bottomLeft)
            image = draw_quad_centerpoint(image, topLeft, topRight, bottomRight, bottomLeft)
            image = draw_quad_markerID(image, topLeft, topRight, bottomRight, bottomLeft, markerID)

        # Choose the first tag detected for warping function
        (topLeft, topRight, bottomRight, bottomLeft) = extract_marker_corners(corners[0])
        # Warp perspective to make a tag square
        image = warp_quad_to_square(image, topLeft, topRight, bottomRight, bottomLeft)
        return image

image = process_image(image, arucoDict, arucoParams)

# Show image
window_name = 'image'       # Name display window for image
cv2.imshow("Image", image)  # Show the image using cv2.imshow() method
cv2.waitKey(0)              # wait for user key press (necessary to avoid Python kernel form crashing)
cv2.destroyAllWindows()     # closing all open windows (after key press)
