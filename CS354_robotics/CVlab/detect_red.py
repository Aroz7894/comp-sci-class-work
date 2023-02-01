#!/usr/bin/env python
"""  Brief demo of image processing using OpenCV Python bindings. 

Author: Andrew Rozniakowski
Version: 3/24/2015
"""
import cv2


def find_reddest_pixel(img):
    """ Return the pixel location of the reddest pixel in the image.  
    
       Redness is defined as: redness = (r - g) + (r - b)

       Arguments: 
            img - height x width x 3 numpy array of uint8 values.

       Returns: 
            A tuple (x,y) containg the position of the reddest pixel. 
    """
    redness = 0
    high = redness 
    width = img.shape[1]
    height = img.shape[0]

    print width, height
  
    for x in range(width):
        for y in range(height):
            redness = (int(img[x,y,2]) - int(img[x,y,1])) + (int(img[x,y,2]) - int(img[x,y,0]))
            if(redness > high):
                high = redness
                x_value = x
                y_value = y

    return (x_value, y_value)


def find_reddest_pixel_fast(img):
    """ Return the pixel location of the reddest pixel in the image.  
    
       Redness is defined as: redness = (r - g) + (r - b)

       Arguments: 
            img - height x width x 3 numpy array of uint8 values.

       Returns: 
            A tuple (x,y) containg the position of the reddest pixel. 
    """
    maxVal = 0
    minVal = 0
    minLoc = 0
    maxLoc = 0
    red_array = img[0:][0:][2];
    blue_array = img[0:][0:][1];
    green_array = img[0:][0:][0];
    final_arr = (red_array - blue_array) + (red_array - green_array)
    
    cv2.minMaxLoc(final_array, minVal, maxVal, minLoc, maxLoc)
    
    return maxLoc

def camera_loop():
    """ 
    Find and mark the reddest pixel in the video stream. 
    """
    width = 320
    height = 240
    
    # Tell OpenCV which camera to use:
    capture = cv2.VideoCapture(0)
    
    # Set up image capture to grab the correct size:
    capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)
    
    # See what size image is REALLY being captured 
    # (in case setting failed above.)
    height = capture.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
    width = capture.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)

    while(True):
        # Grab the image from the camera. 
        success, img = capture.read()

        # Find the most-red pixel:
        red_pixel = find_reddest_pixel(img)

        # Draw a circle on the red pixel. 
        # http://docs.opencv.org/modules/core/doc/drawing_functions.html
        cv2.circle(img, red_pixel, 5, (0,255,0), -1)

        cv2.imshow("Image", img)
        c = cv2.waitKey(33)

if __name__ == "__main__":
    camera_loop()
