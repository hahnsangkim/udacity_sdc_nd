import numpy as np
import cv2
import glob
import pickle
import time
from sklearn.svm import LinearSVC
from sklearn.preprocessing import StandardScaler
from skimage.feature import hog
from lesson_functions import *
# NOTE: the next import is only valid for scikit-learn version <= 0.17
# for scikit-learn >= 0.18 use:
from sklearn.model_selection import train_test_split
#from sklearn.cross_validation import train_test_split

### Parameters to be tweaked
paras = {}
paras['color_space'] = 'HLS' # Can be RGB, HSV, LUV, HLS, YUV, YCrCb
paras['orient'] = 9 # HOG orientations
paras['pix_per_cell'] = 8 # HOG pixels per cell
paras['cell_per_block'] = 2 # HOG cells per block
paras['hog_channel'] = 'ALL' # Can be 0, 1, 2, or "ALL"
paras['spatial_size'] = (16, 16) # Spatial binning dimensions
paras['hist_bins'] = 16 # Number of histogram bins
paras['spatial_feat'] = True # Spatial features on or off
paras['hist_feat'] = True # Histogram features on or off
paras['hog_feat'] = True # HOG features on or off

### Train and Classification of LSVC

# Read in cars and notcars
carimages = glob.glob('dat/vehicles/**/*.png', recursive=True)
noncarimages = glob.glob('dat/non-vehicles/**/*.png', recursive=True)
cars = []
notcars = []

for image in carimages:
    cars.append(image)
for image in noncarimages:
    notcars.append(image)
        
car_features = extract_features(cars, color_space=paras['color_space'], 
                        spatial_size=paras['spatial_size'], hist_bins=paras['hist_bins'], 
                        orient=paras['orient'], pix_per_cell=paras['pix_per_cell'], 
                        cell_per_block=paras['cell_per_block'], 
                        hog_channel=paras['hog_channel'], spatial_feat=paras['spatial_feat'], 
                        hist_feat=paras['hist_feat'], hog_feat=paras['hog_feat'])
notcar_features = extract_features(notcars, color_space=paras['color_space'], 
                        spatial_size=paras['spatial_size'], hist_bins=paras['hist_bins'], 
                        orient=paras['orient'], pix_per_cell=paras['pix_per_cell'], 
                        cell_per_block=paras['cell_per_block'], 
                        hog_channel=paras['hog_channel'], spatial_feat=paras['spatial_feat'], 
                        hist_feat=paras['hist_feat'], hog_feat=paras['hog_feat'])

X = np.vstack((car_features, notcar_features)).astype(np.float64)                        
# Fit a per-column scaler
X_scaler = StandardScaler().fit(X)
# Apply the scaler to X
scaled_X = X_scaler.transform(X)

# Define the labels vector
y = np.hstack((np.ones(len(car_features)), np.zeros(len(notcar_features))))


# Split up data into randomized training and test sets
rand_state = np.random.randint(0, 100)
X_train, X_test, y_train, y_test = train_test_split(
    scaled_X, y, test_size=0.2, random_state=rand_state)

print('Using:',paras['orient'],'orientations',paras['pix_per_cell'],
    'pixels per cell and', paras['cell_per_block'],'cells per block')
print('Feature vector length:', len(X_train[0]))
# Use a linear SVC 
svc = LinearSVC()
# Check the training time for the SVC
t=time.time()
svc.fit(X_train, y_train)
t2 = time.time()
print(round(t2-t, 2), 'Seconds to train SVC...')
# Check the score of the SVC
print('Test Accuracy of SVC = ', round(svc.score(X_test, y_test), 4))
# store the classifier
pickle.dump(svc, open("model/mylsvc.pkl", "wb"))
pickle.dump(X_scaler, open("model/myxscaler.pkl", "wb"))
pickle.dump(paras, open("model/myparas.pkl", "wb"))
print('Stored the classifier, Xscaler, and parameters!')
