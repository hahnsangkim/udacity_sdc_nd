### Prediction: Detecting Vehicles
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import cv2
import glob
import pickle
import time
from sklearn.svm import LinearSVC
from sklearn.preprocessing import StandardScaler
from skimage.feature import hog
from lesson_functions import *
from defined_functions import *
# NOTE: the next import is only valid for scikit-learn version <= 0.17
# for scikit-learn >= 0.18 use:
from sklearn.model_selection import train_test_split

# Check the prediction time for a single sample
t=time.time()

svc = pickle.load(open("model/mylsvc.pkl", "rb") )
X_scaler = pickle.load(open("model/myxscaler.pkl", "rb"))
paras = pickle.load(open("model/myparas.pkl", "rb"))
print('Loaded the classifier, Xscaler, and parameters!')

image = mpimg.imread('bbox-example-image.jpg')
draw_image = np.copy(image)

# Uncomment the following line if you extracted training
# data from .png images (scaled 0 to 1 by mpimg) and the
# image you are searching is a .jpg (scaled 0 to 255)
image = image.astype(np.float32)/255

small_windows = slide_window(image, x_start_stop=[None, None], y_start_stop=[490, 560], 
                    xy_window=(64, 64), xy_overlap=(0.5, 0.5))

mid_windows = slide_window(image, x_start_stop=[None, None], y_start_stop=[490, 650], 
                    xy_window=(128, 128), xy_overlap=(0.6, 0.6))

large_windows = slide_window(image, x_start_stop=[None, None], y_start_stop=[490, 720], 
                    xy_window=(192, 192), xy_overlap=(0.8, 0.8))

xlarge_windows = slide_window(image, x_start_stop=[None, None], y_start_stop=[490, 700], 
                    xy_window=(320, 192), xy_overlap=(0.8, 0.8))

windows = small_windows + mid_windows + large_windows + xlarge_windows

hot_windows = search_windows(image, windows, svc, X_scaler, color_space=paras['color_space'], 
                             spatial_size=paras['spatial_size'], hist_bins=paras['hist_bins'], 
                             orient=paras['orient'], pix_per_cell=paras['pix_per_cell'], 
                             cell_per_block=paras['cell_per_block'], 
                             hog_channel=paras['hog_channel'], spatial_feat=paras['spatial_feat'], 
                             hist_feat=paras['hist_feat'], hog_feat=paras['hog_feat'])                       

rect_windows = cleanup_windows(hot_windows)

t2 = time.time()
print(round(t2-t, 2), 'Seconds to predict...')

window_img = draw_boxes(draw_image, rect_windows, color=(0, 0, 255), thick=6)                    

plt.imsave('output_images/bbox-example-image_detected.jpg', window_img)
