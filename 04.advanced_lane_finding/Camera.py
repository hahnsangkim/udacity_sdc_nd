import numpy as np
import cv2
import glob
import pickle
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.gridspec as gridspec


class Camera:
    def __init__(self, debug_mode = False, show_plots = False):
        self.debug_mode = debug_mode
        self.show_plots = show_plots
        self.mtx = None
        self.dist = None
        self.cali_loaded = False
        with open("calibration_wide/wide_dist_pickle.p", "rb") as ofile:
            print("mtx and dist loaded")
            dist_pickle = pickle.load(ofile)
            self.mtx = dist_pickle["mtx"]
            self.dist = dist_pickle["dist"]
            self.cali_loaded = True
        self.rvecs = None
        self.tvecs = None
        # Source coords for perspective xform
        #([[250,719], [565,460], [676,460], [1116,719]])
        self.src = np.float32([[203,720], [585,460], [695,460], [1127,720]])
        # Dest coords for perspective xform
        self.dst = np.float32([[320,720], [320,0], [960,0], [960,720]])
        # Perspective Transform matrix
        self.M = cv2.getPerspectiveTransform(self.src, self.dst)
        # Inverse Perspective Transform matrix
        self.Minv = cv2.getPerspectiveTransform(self.dst, self.src)
        '''
        src = np.float32(
        [[(img_size[0] / 2) - 55, img_size[1] / 2 + 100],
        [((img_size[0] / 6) - 10), img_size[1]],
        [(img_size[0] * 5 / 6) + 60, img_size[1]],
        [(img_size[0] / 2 + 55), img_size[1] / 2 + 100]])
        dst = np.float32(
        [[(img_size[0] / 4), 0],
        [(img_size[0] / 4), img_size[1]],
        [(img_size[0] * 3 / 4), img_size[1]],
        [(img_size[0] * 3 / 4), 0]])
        Source 	Destination
        585, 460 	320, 0
        203, 720 	320, 720
        1127, 720 	960, 720
        695, 460 	960, 0
        '''

    def findTheBestXY(self, images):
        maxcount = 0
        X, Y = 0, 0
        for x in range(3, 10):
            for y in range(3, 7):
                count = 0
                for fname in images:
                    img = mpimg.imread(fname)
                    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                    ret, corners = cv2.findChessboardCorners(gray, (x,y),None)
                    if ret == True:
                        count += 1
                if maxcount < count:
                    maxcount = count
                    X, Y = x, y
        return X, Y, maxcount    

    def calibrate(self, imgfiles):
        # Make a list of calibration images
        if self.cali_loaded:
            return True
        images = glob.glob(imgfiles)
        X, Y, maxcount = self.findTheBestXY(images)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((X*Y, 3), np.float32)
        objp[:,:2] = np.mgrid[0:X,0:Y].T.reshape(-1,2)
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d points in real world space
        imgpoints = [] # 2d points in image plane.
        corners_found = False
        for fname in images:
            img = mpimg.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (X-1,Y-1),None)
            if ret == True:
                imgpoints.append(corners)
                objpoints.append(objp)
                # Draw and display the corners
                cv2.drawChessboardCorners(img, (X-1,Y-1), corners, ret)
                if debug_mode:
                    plt.imshow(img)
                corners_found = True
            if corners_found:
                img_size = (img.shape[1], img.shape[0])
                # Do camera calibration given object points and image points
                ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)
                #undist = cv2.undistort(img, self.mtx, self.dist, None, self.mtx)
                dist_pickle = {}
                dist_pickle["mtx"] = self.mtx
                dist_pickle["dist"] = self.dist
                pickle.dump( dist_pickle, open( "calibration_wide/wide_dist_pickle.p", "wb" ) )
                self.cali_loaded = True


    def undistort(self, img):
        undist = cv2.undistort(img, self.mtx, self.dist, None, self.mtx)
        if self.show_plots:
            f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20,10))
            ax1.imshow(img);
            ax1.set_title('Original Image', fontsize=30)
            ax2.imshow(undist);
            ax2.set_title('1. Undistorted Image', fontsize=30)
        return undist
    

    def colorGradientThreshold(self, img):
        # Convert to HLS color space and separate the S channel
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        s_channel = hls[:,:,2]
        # Grayscale image
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        # Sobel x
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0) # Take the derivative in x
        abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
        # Threshold x gradient
        thresh_min = 30
        thresh_max = 150
        sxbinary = np.zeros_like(scaled_sobel)
        #sxbinary[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1
        retval, sxthresh = cv2.threshold(scaled_sobel, thresh_min, thresh_max, cv2.THRESH_BINARY)
        sxbinary[(sxthresh >= thresh_min) & (sxthresh <= thresh_max)] = 1
        if self.show_plots:
            plt.imshow(sxthresh)
            plt.title('sxthresh')
            plt.show()
            plt.imshow(sxbinary)
            plt.title('2.1. Threshold Gradient')
            plt.show()
        # Threshold color channel
        s_thresh_min = 175
        s_thresh_max = 255
        s_binary = np.zeros_like(s_channel)
        #s_binary[(s_channel >= s_thresh_min) & (s_channel <= s_thresh_max)] = 1
        s_thresh = cv2.inRange(s_channel.astype('uint8'), s_thresh_min, s_thresh_max)
        s_binary[(s_thresh == 255)] = 1
        if self.show_plots:
            plt.imshow(s_thresh)
            plt.title('s_thresh')
            plt.show()
            plt.imshow(s_binary)
            plt.title('2.2. Threshold Binary')
            plt.show()
        # Stack each channel to view their individual contributions in green and blue respectively
        # This returns a stack of the two binary images, whose components you can see as different colors
        if self.show_plots:
            color_binary = np.dstack(( np.zeros_like(sxthresh), sxthresh, s_thresh))
            plt.imshow(color_binary)
            plt.title('Color Binary')
            plt.show()
        # Combine the two binary thresholds
        combined_binary = np.zeros_like(sxbinary)
        combined_binary[(s_binary == 1) | (sxbinary == 1)] = 1
        # Plotting thresholded images
        if self.show_plots:
            f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20,10))
            ax1.set_title('Stacked thresholds')
            ax1.imshow(color_binary)
            ax2.set_title('3. Combined S channel and gradient thresholds')
            ax2.imshow(combined_binary, cmap='gray')
            plt.show()
        return combined_binary


    def warpPerspective(self, img):
        img_size = (img.shape[1], img.shape[0])
        perspective_img = cv2.warpPerspective(img, self.M, img_size, flags=cv2.INTER_LINEAR)
        if self.show_plots:
            f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20,10))
            ax1.set_title('3. Combined S channel and gradient thresholds')
            ax1.imshow(img, cmap='gray')
            ax2.set_title('4. Perspective Transformed')
            ax2.imshow(perspective_img, cmap='gray')
            plt.show()
        return perspective_img

    def unwarpPerspective(self, org, warped, ploty, left_fitx, right_fitx):
        warp_zero = np.zeros_like(warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, self.Minv, (org.shape[1], org.shape[0])) 
        return newwarp
