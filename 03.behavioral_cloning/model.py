import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import random
import math
import json
import cv2
import os
from keras.models import Sequential, model_from_json
from keras.layers import Flatten, Dense, Lambda, Cropping2D
from keras.layers.convolutional import Convolution2D
from keras.layers.normalization import BatchNormalization
from keras.callbacks import ModelCheckpoint
from keras.optimizers import Adam
from keras.regularizers import l2
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from pathlib import Path

################
steering_adjustment =  0.27

debug = True
df = pd.read_csv("./data/driving_log.csv", delimiter=',')
#columns = ['center', 'left', 'right', 'steering', 'throttle', 'brake', 'speed']
#df['steering'].apply(pd.to_numeric)
#df['brake'].apply(pd.to_numeric)
#df['speed'].apply(pd.to_numeric)
df = df[(df.brake==0.0) & (df.speed > 0.0)]
center = df.center.tolist()
center_re = df.center.tolist()
left = df.left.tolist()
right = df.right.tolist()
steering = df.steering.tolist()
steering_re = df.steering.tolist()

## SPLIT TRAIN AND VALID ##
center, steering = shuffle(center, steering)
center, X_valid, steering, y_valid = train_test_split(center, steering, test_size = 0.10, random_state = 100)

d_center, d_left, d_right = [], [], []
a_center, a_left, a_right = [], [], []
for i in steering:
  #Positive angle is turning from Left -> Right. Negative is turning from Right -> Left#
  index = steering.index(i)
  if i > 0.15:
    d_right.append(center[index])
    a_right.append(i)
  if i < -0.15:
    d_left.append(center[index])
    a_left.append(i)
  else:
    d_center.append(center[index])
    a_center.append(i)

## ADD RECOVERY ##
#  Find the amount of sample differences between driving straight & driving left, driving straight & driving right #
ds_size, dl_size, dr_size = len(d_center), len(d_left), len(d_right)
main_size = math.ceil(len(center_re))
l_xtra = ds_size - dl_size
r_xtra = ds_size - dr_size
# Generate random list of indices for left and right recovery images
indice_L = random.sample(range(main_size), l_xtra)
indice_R = random.sample(range(main_size), r_xtra)

# Filter angle less than -0.15 and add right camera images into driving left list, minus an adjustment angle #
for i in indice_L:
  if steering_re[i] < -0.15:
    d_left.append(right[i])
    a_left.append(steering_re[i] - steering_adjustment)

# Filter angle more than 0.15 and add left camera images into driving right list, add an adjustment angle #
for i in indice_R:
  if steering_re[i] > 0.15:
    d_right.append(left[i])
    a_right.append(steering_re[i] + steering_adjustment)

## COMBINE TRAINING IMAGE NAMES AND ANGLES INTO X_train and y_train ##
X_train = d_center + d_left + d_right
y_train = np.float32(a_center + a_left + a_right)
assert len(X_train) == len(y_train), "Found input variables with inconsistent numbers of samples:[{}, {}]".format(len(X_train),len(y_train))
'''
###########
import csv

samples = []
with open('./data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    next(reader, None)
    for line in reader:
        samples.append(line)

samples = shuffle(samples)
train_samples, validation_samples = train_test_split(samples, test_size=0.1, random_state=10)
print("# of train samples: {}, # of validation samples: {}".format(len(train_samples), len(validation_samples)))
assert len(samples) == (len(train_samples)+len(validation_samples)), "Total number of samples should equal to the sum of train's and validation's"
'''
################
augment = 6
batch_size = 128
samples_per_epoch = math.ceil(len(X_train)/batch_size)*batch_size
nb_val_samples = len(X_valid)
nb_epoch = 60
verbose = 2
input_shape = (160,320,3) #(img_rows, img_cols, ch)
################
#assert samples_per_epoch == len(samples), "samples_per_epoch should equal the total number of samples to be processed"
# Generate random brightness function, produce darker transformation
def random_brightness(img):
    #Convert 2 HSV colorspace from RGB colorspace
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #Generate new random brightness
    rand = random.uniform(0.3,1.0)
    hsv[:,:,2] = rand*hsv[:,:,2]
    #Convert back to RGB colorspace
    new_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return new_img

def generator_train(X_train, y_train, batch_size):
    shape = [x for x in input_shape]
    shape.insert(0, batch_size)
    batch_train = np.zeros(tuple(shape), dtype = np.float32)
    batch_angle = np.zeros((batch_size,), dtype = np.float32)
    while True:
        data, angle = shuffle(X_train, y_train)
        for i in range(batch_size):
            choice = int(np.random.choice(len(data),1))
            fname = data[choice].replace(' ','').split('/')
            #token = fname.split('/')
            batch_train[i] = random_brightness(cv2.imread('./data/IMG/'+fname[-1]))
            batch_angle[i] = angle[choice]*(1+ np.random.uniform(-0.10,0.10))
            #Flip random images#
            flip_coin = random.randint(0,1)
            if flip_coin == 1:
                batch_train[i] = cv2.flip(batch_train[i], 1)
                batch_angle[i] = batch_angle[i]*(-1.0)
        yield batch_train, batch_angle

# Validation generator: pick random samples. Apply resizing and cropping on chosen samples
def generator_valid(data, angle, batch_size):
    shape = [x for x in input_shape]
    shape.insert(0, batch_size)
    batch_train = np.zeros(tuple(shape), dtype = np.float32)
    batch_angle = np.zeros((batch_size,), dtype = np.float32)
    while True:
        data, angle = shuffle(data,angle)
        for i in range(batch_size):
            rand = int(np.random.choice(len(data),1))
            fname = './data/IMG/'+data[rand].replace(' ','')
            token = fname.split('/')
            data[rand] = token[-1]
            batch_train[i] = cv2.imread(data[rand])
            batch_angle[i] = angle[rand]
        yield batch_train, batch_angle

'''
def generator(samples, batch_size):
    while 1: # Loop forever so the generator never terminates
        #samples = shuffle(samples)
        corrections = [0, -0.2, 0.2]
        for offset in range(0, len(samples), batch_size):
            batch_samples = samples[offset:offset+batch_size]
            images = []
            angles = []
            for batch_sample in batch_samples:
                for i in range(3):
                    fname = batch_sample[i].replace(' ', '')
                    name = './data/IMG/'+fname.split('/')[-1]
                    image = cv2.imread(name)
                    angle = float(batch_sample[3]) + corrections[i]
                    images.append(image)
                    angles.append(angle)
                    #augmenting data
                    flip_coin = random.randint(0, 1)
                    if flip_coin == 1:
                        image = np.fliplr(image)
                        angle = angle * -1.0

                    images.append(image_flipped)
                    angles.append(angle_flipped)

            # trim image to only see section with road
            X_train = np.array(images)
            y_train = np.array(angles)
            yield shuffle(X_train, y_train)
'''

# compile and train the model using the generator function
train_generator = generator_train(X_train, y_train, batch_size)
validation_generator = generator_valid(X_valid, y_valid, batch_size)

modelJson = 'model.json'
modelWeights = 'model.h5'
path = './model/'
adam = Adam(lr=0.0001)
# load model.json and model.h5 if they exists
if Path(modelJson).is_file():
    with open(modelJson, 'r') as jfile:
        models = model_from_json(json.load(jfile))
    # load weights into new model
    models.compile(optimizer=adam, loss="mse", metrics=['accuracy'])
    models.load_weights(modelWeights)
    print("Loaded model from disk:")
# re-create our model and restart training.
else:
    models = Sequential()
    models.add(Cropping2D(cropping=((50,20), (1,1)), input_shape=input_shape))
    models.add(Lambda(lambda x: x/255.0 - 0.5))
    models.add(Convolution2D(24,5,5, subsample=(2,2), activation='relu', W_regularizer = l2(0.001)))
    models.add(BatchNormalization())
    models.add(Convolution2D(36,5,5, subsample=(2,2), activation='relu', W_regularizer = l2(0.001)))
    models.add(BatchNormalization())
    models.add(Convolution2D(48,5,5, subsample=(2,2), activation='relu', W_regularizer = l2(0.001)))
    models.add(BatchNormalization())
    models.add(Convolution2D(64,3,3,activation='relu', W_regularizer = l2(0.001)))
    models.add(BatchNormalization())
    models.add(Convolution2D(64,3,3, activation='relu', W_regularizer = l2(0.001)))
    models.add(BatchNormalization())
    #models.add(MaxPooling2D())
    models.add(Flatten())
    models.add(Dense(100, W_regularizer = l2(0.001)))
    #models.add(BatchNormalization())
    models.add(Dense(50, W_regularizer = l2(0.001)))
    #models.add(BatchNormalization())
    models.add(Dense(10, W_regularizer = l2(0.001)))
    #models.add(BatchNormalization())
    models.add(Dense(1, W_regularizer = l2(0.001)))
    models.compile(optimizer=adam, loss='mse', metrics=['accuracy'])
models.summary()

#checkpoint
filepath="model/weights.{epoch:02d}-{val_loss:.2f}.hdf5"
checkpoint = ModelCheckpoint(filepath, monitor='val_acc', verbose=1, save_best_only=True, mode='max')
callbacks_list = [checkpoint]
history_object = models.fit_generator(train_generator, samples_per_epoch = samples_per_epoch,
                                      callbacks=callbacks_list, validation_data =  validation_generator, 
                                      nb_val_samples = nb_val_samples, nb_epoch=nb_epoch, verbose=verbose)

# save model and weights to disk.
print("Saving model and weights to disk: ",modelJson,"and",modelWeights)
if Path(modelJson).is_file():
    os.remove(modelJson)
json_string = models.to_json()
with open(modelJson,'w' ) as f:
    json.dump(json_string, f)
if Path(modelWeights).is_file():
    os.remove(modelWeights)
models.save_weights(modelWeights)

### print the keys contained in the history object
print(history_object.history.keys())
### plot the training and validation loss for each epoch
plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()
exit()
