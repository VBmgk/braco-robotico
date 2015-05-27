#coding: UTF-8

'''multi_color_multiple_points_track.py

In single_color_multiple_points_track.py, we tracked multiple points of same color. Now you can see with a very small modification, we can track multiple points with different colors. It is very simple.

I have just commented out draw_line part. If you want uncomment it. Then you will get a polygon connecting all these points.
For example, a triangle if only three points ( like i got in multi_color.png)

    
Written by Abid.K   --mail me at abidrahman2@gmail.com  '''

###############################################################################################################################

import cv

# The color ranges are in HSV space, not RGB. 
""" Keep in mind that different applications use different scales for HSV. For example, GIMP uses:
        H: 0-360, S: 0-100, V: 0-100
    While OpenCv uses:
        H: 0-180, S: 0-255, V: 0-255
    So you can use GIMP to analyze a sample image and find out what colors you want, and then transform to OpenCv scale.
"""

delta = (5, 50, 50)

orange = cv.Scalar(13,205,205)
red = cv.Scalar(0, 205,205)
green = cv.Scalar(68,195,167)

colors = {
        'orange': orange,
        'green': green, 
        'red': red
    }

px = 0 
py = 300 

def run():
    import time
    from math import pi

    capture=cv.CaptureFromCAM(1)
    # Pego essa imagem só pra saber o tamanho das imagens
    im = cv.QueryFrame(capture)
    im_height = im.height
    im_width = im.width

    cv.NamedWindow("Real", 0)
    for color in colors.keys():
        cv.NamedWindow(color, 0)
    # XXX Tirar esse while caso queira detectar apenas uma vez
    while(True):
        for k,v in colors.iteritems():
            centroid = run_from_cam(capture, {k:v})[k]
            if centroid:
                centroid = centroid[0]
                position = transformation(pi, centroid[0]/float(im_width), centroid[1]/float(im_height), centroid[0], centroid[1], px, py)
            else:
                position = ()
            print '{}: {}, {}'.format(k, centroid, position)
        print
        # XXX Mudar aqui o tempo de espera até pegar o próximo frame
        time.sleep(1)

def transformation(theta, alpha, beta, cx, cy, px, py):
    from math import cos, sin
    w = cos(theta)*alpha*cx - sin(theta)*beta*cy + px
    h = sin(theta)*alpha*cx + cos(theta)*beta*cy + py

    return (w,h)

def getthresholdedimg(im, color):
    '''this function take RGB image.Then convert it into HSV for easy colour detection and threshold it with the color chosen as white and all other regions as black.Then return that image'''
    imghsv=cv.CreateImage(cv.GetSize(im),8,3)
    cv.CvtColor(im,imghsv,cv.CV_BGR2HSV)                # Convert image from RGB to HSV
        
    # A little change here. Creates images for green,blue and yellow (or whatever color you like).
    imgcolor=cv.CreateImage(cv.GetSize(im),8,1)
    imgthreshold=cv.CreateImage(cv.GetSize(im),8,1)

    color_low_threshold = (color[0] - delta[0], color[1] - delta[1], color[2] - delta[2])
    color_high_threshold = (color[0] + delta[0], color[1] + delta[1], color[2] + delta[2])
    
    cv.InRangeS(imghsv,color_low_threshold,color_high_threshold,imgcolor) 
    cv.Add(imgthreshold,imgcolor,imgthreshold)
    return imgthreshold

def run_from_img(original_image, colors):
    """ Receives an image and returns the centroids of the areas detected with the specified color.
        To load an image and pass to this function:
            im = cv.LoadImage("image.png")
    """
    centroids = {}
    thresholded_imgs = {}
    imgthresh = {}
    color_image = {}
    storage = {}
    contour = {}

    for color in colors.keys():
        color_image[color] = cv.CloneImage(original_image)
        cv.Smooth(color_image[color], color_image[color], cv.CV_GAUSSIAN, 3, 0)

        imgthresh[color]=getthresholdedimg(color_image[color], colors[color])
        thresholded_imgs[color] = cv.CloneImage(imgthresh[color])

        cv.Erode(imgthresh[color],imgthresh[color],None,3)
        cv.Dilate(imgthresh[color],imgthresh[color],None,10)
        storage[color] = cv.CreateMemStorage(0)
        contour[color] = cv.FindContours(imgthresh[color], storage[color], cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)

        centroids[color] = []
        while contour[color]:
            # Centroid calculation
            moment = cv.Moments(contour[color])
            Cx = int(moment.m10 / moment.m00)
            Cy = int(moment.m01 / moment.m00)

            centroids[color].append((Cx, Cy))

            contour[color] = contour[color].h_next()

    return centroids, thresholded_imgs

def run_from_cam(capture, colors):
    """ This fuction keeps reading frames from the CAM and passing them to the function run_from_img, for color detection, and printing the centroids found.
    """
    #import time

    #capture=cv.CaptureFromCAM(0)

    #while(1):
    color_image = cv.QueryFrame(capture)
    # Mirror the image from the cam
    cv.Flip(color_image,color_image,1)

    centroids, thresholded_imgs = run_from_img(color_image, colors)

    # Para salvar as imagens em arquivos
    #for color in colors.keys():
    #    cv.SaveImage("Real_"+color+".png", color_image)
    #    cv.SaveImage(color+".png", thresholded_imgs[color])

    # Para mostrar as imagens em janelas
    cv.ShowImage("Real", color_image)
    for color in colors.keys():
        cv.ShowImage(color, thresholded_imgs[color])

    # Isso é necessário para que as janelas apareçam
    if cv.WaitKey(33)==1048603:
        cv.DestroyWindow("Real")
        for color in colors.keys():
            cv.DestroyWindow(color)

    return centroids

    #time.sleep(0.5)
