from scipy.ndimage.measurements import label
from skimage.feature import blob_doh
from scipy import ndimage as ndi
from skimage.morphology import watershed
from skimage.color import rgb2gray
import numpy as np
import pickle
from lesson_functions import *
# Import everything needed to edit/save/watch video clips
import imageio
imageio.plugins.ffmpeg.download()
from moviepy.editor import VideoFileClip

def single_img_features(img, color_space='RGB', spatial_size=(32, 32),
                        hist_bins=32, orient=9, 
                        pix_per_cell=8, cell_per_block=2, hog_channel=0,
                        spatial_feat=True, hist_feat=True, hog_feat=True):  
    '''
    The function extracts features from a single image window
    very similar to extract_features() just for a single image 
    rather than list of images
    '''
    #1) Define an empty list to receive features
    img_features = []
    #2) Apply color conversion if other than 'RGB'
    if color_space != 'RGB':
        if color_space == 'HSV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        elif color_space == 'LUV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
        elif color_space == 'HLS':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        elif color_space == 'YUV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
        elif color_space == 'YCrCb':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)
    else: feature_image = np.copy(img)      
    #3) Compute spatial features if flag is set
    if spatial_feat == True:
        spatial_features = bin_spatial(feature_image, size=spatial_size)
        #4) Append features to list
        img_features.append(spatial_features)
    #5) Compute histogram features if flag is set
    if hist_feat == True:
        hist_features = color_hist(feature_image, nbins=hist_bins)
        #6) Append features to list
        img_features.append(hist_features)
    #7) Compute HOG features if flag is set
    if hog_feat == True:
        if hog_channel == 'ALL':
            hog_features = []
            for channel in range(feature_image.shape[2]):
                hog_features.append(get_hog_features(feature_image[:,:,channel], 
                                    orient, pix_per_cell, cell_per_block, 
                                    vis=False, feature_vec=False))      
            hog_features = np.ravel(hog_features)
        else:
            hog_features = get_hog_features(feature_image[:,:,hog_channel], orient, 
                        pix_per_cell, cell_per_block, vis=False, feature_vec=False)
        #8) Append features to list
        img_features.append(hog_features)

    #9) Return concatenated array of features
    return np.concatenate(img_features)

def search_windows(img, windows, clf, scaler, color_space='RGB', 
                   spatial_size=(32, 32), hist_bins=32, 
                   hist_range=(0, 256), orient=9, 
                   pix_per_cell=8, cell_per_block=2, 
                   hog_channel=0, spatial_feat=True, 
                   hist_feat=True, hog_feat=True):
    '''
    The function returns the list of windows to be searched
    '''
    #1) Create an empty list to receive positive detection windows
    on_windows = []
    #2) Iterate over all windows in the list
    for window in windows:
        #3) Extract the test window from original image
        test_img = cv2.resize(img[window[0][1]:window[1][1], window[0][0]:window[1][0]], (64, 64))      
        #4) Extract features for that window using single_img_features()
        features = single_img_features(test_img, color_space=color_space, 
                                       spatial_size=spatial_size, hist_bins=hist_bins, 
                                       orient=orient, pix_per_cell=pix_per_cell, 
                                       cell_per_block=cell_per_block, 
                                       hog_channel=hog_channel, spatial_feat=spatial_feat,
                                       hist_feat=hist_feat, hog_feat=hog_feat)
        #5) Scale extracted features to be fed to classifier
        test_features = scaler.transform(np.array(features).reshape(1, -1))
        #6) Predict using your classifier
        prediction = clf.predict(test_features)
        #7) If positive (prediction == 1) then save the window
        if prediction == 1:
            on_windows.append(window)
    #8) Return windows for positive detections
    return on_windows

def add_heat(heatmap, bboxlist):
    # Iterate through list of bboxes
    for box in bboxlist:
        # Add += 1 for all pixels inside each bbox
        heatmap[box[0][1]:box[1][1], box[0][0]:box[1][0]] += 1
    
    # Return updated heatmap
    return heatmap
    
def apply_threshold(heatmap, threshold):
    # Zero out pixels below the threshold
    heatmap[heatmap <= threshold] = 0
    # Return thresholded map
    return heatmap

def draw_labeled_bboxes(img, labels):
    # Iterate through all detected cars
    for car_number in range(1, labels[1]+1):
        # Find pixels with each car_number label value
        nonzero = (labels[0] == car_number).nonzero()
        #print(nonzero)
        # (array([456, 456, 456, ..., 519, 519, 519]), array([546, 547, 548, ..., 583, 584, 585]))
        # Identify x and y values of those pixels
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Define a bounding box based on min/max x and y
        bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
        #reclist, weights = cv2.groupRectangles(np.array(rect).tolist(), groupThreshold = 1, eps = 0.2)
        # Draw the box on the image
        cv2.rectangle(img, bbox[0], bbox[1], (0,0,255), 6)
    # Return the image
    return img

def process_image(image):
    global svc
    global X_scaler
    global paras
    global all_boxes

    imgwidth = image.shape[1]
    imgheight = image.shape[0]
    
    botoffset = 30
    maxrec = int(imgwidth/4)
    minrec = 64
    mid1rec = int((maxrec + minrec)*2/3)
    mid2rec = int((maxrec + minrec)*1/3)
    #print(maxrec, mid1rec, mid2rec, minrec)

    center = imgheight-botoffset-int(maxrec/2)
    ystart = center-int(maxrec/2)
    ystop = center+int(maxrec/2)
    maxyss = [ystart, ystop]
    center = center - int(maxrec/4)
    ystart = center - int(mid1rec/2)
    ystop = center + int(mid1rec/2)
    mid1yss = [ystart, ystop]
    ystart = center - int(mid2rec/2)
    ystop = center + int(mid2rec/2)
    mid2yss = [ystart, ystop]
    ystart = center - int(minrec/2)
    ystop = center + int(minrec)
    minyss = [ystart, ystop]
    #print(maxyss, mid1yss, mid2yss, minyss)
    maxxss = [0, imgwidth]
    mid1xss = [mid1rec, imgwidth]
    mid2xss = [int(imgwidth/5), imgwidth]
    minxss = [int(imgwidth/3), imgwidth]
    #print(maxxss, mid1xss, mid2xss, minxss)

    max_windows = slide_window(image, x_start_stop=maxxss, y_start_stop=maxyss, 
                        xy_window=(maxrec, maxrec), xy_overlap=(0.5, 0.5))
    mid1_windows = slide_window(image, x_start_stop=mid1xss, y_start_stop=mid1yss, 
                        xy_window=(mid1rec, mid1rec), xy_overlap=(0.9, 0.5))
    mid2_windows = slide_window(image, x_start_stop=mid2xss, y_start_stop=mid2yss, 
                        xy_window=(mid2rec, mid2rec), xy_overlap=(0.9, 0.7))
    min_windows = slide_window(image, x_start_stop=minxss, y_start_stop=minyss, 
                        xy_window=(minrec, minrec), xy_overlap=(0.9, 0.7))
    windows = max_windows + mid1_windows + mid2_windows + min_windows 

    #draw_image = np.copy(image)
    #image = image.astype(np.float32)/255
    hot_windows = search_windows(image, windows, svc, X_scaler, color_space=paras['color_space'], 
                             spatial_size=paras['spatial_size'], hist_bins=paras['hist_bins'], 
                             orient=paras['orient'], pix_per_cell=paras['pix_per_cell'], 
                             cell_per_block=paras['cell_per_block'], 
                             hog_channel=paras['hog_channel'], spatial_feat=paras['spatial_feat'], 
                             hist_feat=paras['hist_feat'], hog_feat=paras['hog_feat'])            
    #rect_windows = cleanup_windows(hot_windows)
    heatmap = np.zeros_like(image[:,:,0]).astype(np.float)
    if hot_windows:
        all_bboxes.insert(0, hot_windows)
        if len(all_bboxes) > 8:
            all_bboxes.pop()
        for idx, bboxlist in enumerate(all_bboxes):
            heatmap = add_heat(heatmap, bboxlist)
        heatmap = apply_threshold(heatmap, 3)
    #heat = rgb2gray(heatmap)
    labels = None
    draw_img = None
    blobapplied = False
    if heatmap.any():
        if blobapplied:
            blobs = blob_doh(heatmap)
            blobs = blobs.astype(np.uint32)
            print(blobs)
            centroids = np.zeros(heatmap.shape, dtype=np.bool)
            centroids[blobs[:, 0], blobs[:, 1]] = True
            markers = ndi.label(centroids)[0]
            labels = watershed(-heatmap, markers, mask=heatmap)
            labels = label(labels)
        else:
            labels = label(heatmap)
        draw_img = draw_labeled_bboxes(np.copy(image), labels)
    else:
        draw_img = image
        
    return draw_img

all_bboxes = []
svc = pickle.load(open("model/mylsvc.pkl", "rb") )
X_scaler = pickle.load(open("model/myxscaler.pkl", "rb"))
paras = pickle.load(open("model/myparas.pkl", "rb"))
print('Loaded the classifier, Xscaler, and parameters!')

project_output = './output_images/project_video_boxed.mp4'
clip1 = VideoFileClip("project_video.mp4")
project_clip = clip1.fl_image(process_image) #NOTE: this function expects color images!!
project_clip.write_videofile(project_output, audio=False)
pickle.dump(all_bboxes, open("model/hotbboxes.pkl", "wb"))
