'''multi_color_multiple_points_track.py

In single_color_multiple_points_track.py, we tracked multiple points of same color. Now you can see with a very small modification, we can track multiple points with different colors. It is very simple.

I have just commented out draw_line part. If you want uncomment it. Then you will get a polygon connecting all these points.
For example, a triangle if only three points ( like i got in multi_color.png)

    
Written by Abid.K   --mail me at abidrahman2@gmail.com  '''

###############################################################################################################################

import cv
#posx=0
#posy=0

# The color ranges are in HSV space, not RGB. 
""" Keep in mind that different applications use different scales for HSV. For example, GIMP uses:
        H: 0-360, S: 0-100, V: 0-100
    While OpenCv uses:
        H: 0-180, S: 0-255, V: 0-255
    So you can use GIMP to analyze a sample image and find out what colors you want, and then transform to OpenCv scale.
"""
orange_low = cv.Scalar(3,150,150)
orange_high = cv.Scalar(13,255,255)

blue_low = cv.Scalar(110,150,150)
blue_high = cv.Scalar(130,255,255) 

colors = {
        'orange': (orange_low, orange_high),
        'blue': (blue_low, blue_high)
    }

def getthresholdedimg(im, color_low_threshold, color_high_threshold):
    '''this function take RGB image.Then convert it into HSV for easy colour detection and threshold it with the color chosen as white and all other regions as black.Then return that image'''
    imghsv=cv.CreateImage(cv.GetSize(im),8,3)
    cv.CvtColor(im,imghsv,cv.CV_BGR2HSV)                # Convert image from RGB to HSV
        
    # A little change here. Creates images for green,blue and yellow (or whatever color you like).
    imgcolor=cv.CreateImage(cv.GetSize(im),8,1)
    imgthreshold=cv.CreateImage(cv.GetSize(im),8,1)
    
    cv.InRangeS(imghsv,color_low_threshold,color_high_threshold,imgcolor) 
    cv.Add(imgthreshold,imgcolor,imgthreshold)
    return imgthreshold

def run_from_img(original_image):
    """ Receives an image and returns the centroids of the areas detected with the specified color.
        To load an image and pass to this function:
            im = cv.LoadImage("image.png")
    """
    centroids = {}

    for color_name, color_limits in colors.iteritems():
        color_image = cv.CloneImage(original_image)
        cv.Smooth(color_image, color_image, cv.CV_GAUSSIAN, 3, 0)
        imgyellowthresh=getthresholdedimg(color_image, color_limits[0], color_limits[1])

        cv.Erode(imgyellowthresh,imgyellowthresh,None,3)
        cv.Dilate(imgyellowthresh,imgyellowthresh,None,5)
        storage = cv.CreateMemStorage(0)
        contour = cv.FindContours(imgyellowthresh, storage, cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)

        centroids[color_name] = []
        while contour:
            # Centroid calculation
            moment = cv.Moments(contour)
            Cx = int(moment.m10 / moment.m00)
            Cy = int(moment.m01 / moment.m00)

            centroids[color_name].append((Cx, Cy))

            contour = contour.h_next()

    return centroids

def run_from_cam():
    """ This fuction keeps reading frames from the CAM and passing them to the function run_from_img, for color detection, and printing the centroids found.
    """
    capture=cv.CaptureFromCAM(0)
    while(1):
        color_image = cv.QueryFrame(capture)
        # Mirror the image from the cam
        cv.Flip(color_image,color_image,1)

        centroids = run_from_img(color_image)

        print centroids
