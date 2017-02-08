from urllib.request import urlretrieve
from os.path import isfile
from tqdm import tqdm
import pickle
import numpy as np
import math

print('Modules loaded.')

class DLProgress(tqdm):
    last_block = 0

    def hook(self, block_num=1, block_size=1, total_size=None):
        self.total = total_size
        self.update((block_num - self.last_block) * block_size)
        self.last_block = block_num

if not isfile('train.p'):
    with DLProgress(unit='B', unit_scale=True, miniters=1, desc='Train Dataset') as pbar:
        urlretrieve(
            'https://s3.amazonaws.com/udacity-sdc/datasets/german_traffic_sign_benchmark/train.p',
            'train.p',
            pbar.hook)

if not isfile('test.p'):
    with DLProgress(unit='B', unit_scale=True, miniters=1, desc='Test Dataset') as pbar:
        urlretrieve(
            'https://s3.amazonaws.com/udacity-sdc/datasets/german_traffic_sign_benchmark/test.p',
            'test.p',
            pbar.hook)

print('Training and Test data downloaded.')

################################
# Load Dataset
################################
with open('train.p', 'rb') as f:
    data = pickle.load(f)

# TODO: Load the feature data to the variable X_train
X_train = data['features']
# TODO: Load the label data to the variable y_train
y_train = data['labels']
# STOP: Do not change the tests below. Your implementation should pass these tests. 
assert np.array_equal(X_train, data['features']), 'X_train not set to data[\'features\'].'
assert np.array_equal(y_train, data['labels']), 'y_train not set to data[\'labels\'].'
print('Data checked.')

#############################
# Normalize feature
#############################
def normalize_grayscale(image_data):
    a = -0.5
    b = 0.5
    grayscale_min = 0
    grayscale_max = 255
    return a + ( ( (image_data - grayscale_min)*(b - a) )/( grayscale_max - grayscale_min ) )

X_normalized = normalize_grayscale(X_train)
# STOP: Do not change the tests below. Your implementation should pass these tests. 
assert math.isclose(np.min(X_normalized), -0.5, abs_tol=1e-5) and math.isclose(np.max(X_normalized), 0.5, abs_tol=1e-5), 'The range of the training data is: {} to {}.  It must be -0.5 to 0.5'.format(np.min(X_normalized), np.max(X_normalized))
print('Normalizing features passed.')

###########################
# One-hot Encode labels
###########################
# TODO: One Hot encode the labels to the variable y_one_hot
from sklearn.preprocessing import LabelBinarizer
label_binarizer = LabelBinarizer()
y_one_hot = label_binarizer.fit_transform(y_train)
# STOP: Do not change the tests below. Your implementation should pass these tests. 
import collections

assert y_one_hot.shape == (39209, 43), 'y_one_hot is not the correct shape.  It\'s {}, it should be (39209, 43)'.format(y_one_hot.shape)
assert next((False for y in y_one_hot if collections.Counter(y) != {0: 42, 1: 1}), True), 'y_one_hot not one-hot encoded.'
print('One-hot Encording labels passed.')

#############################
# Split Data
#############################
from sklearn.model_selection import train_test_split
X_train, X_val, y_train, y_val = train_test_split(X_normalized, y_one_hot, test_size=0.25, random_state=42)

#########################
# Build a LeNet
#########################
# TODO: Re-construct the network and add dropout after the pooling layer.
from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D

model = Sequential()
# first set of CONV => RELU => POOL
model.add(Convolution2D(32, 3, 3, border_mode="same", input_shape=(32, 32, 3)))
model.add(Activation("relu"))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
model.add(Dropout(0.5))
# second set of CONV => RELU => POOL
model.add(Convolution2D(50, 3, 3, border_mode="same"))
model.add(Activation("relu"))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
# set of FC => RELU layers
model.add(Flatten())
model.add(Dense(256))
model.add(Activation("relu"))
# softmax classifier
model.add(Dense(43))
model.add(Activation("softmax"))
model.summary()

##########################
# Train the Network
##########################
# STOP: Do not change the tests below. Your implementation should pass these tests.
from keras.layers.core import Dense, Activation, Flatten, Dropout
from keras.activations import relu, softmax
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.optimizers import Adam

def check_layers(layers, true_layers):
    assert len(true_layers) != 0, 'No layers found'
    for layer_i in range(len(layers)):
        assert isinstance(true_layers[layer_i], layers[layer_i]), 'Layer {} is not a {} layer'.format(layer_i+1, layers[layer_i].__name__)
    assert len(true_layers) == len(layers), '{} layers found, should be {} layers'.format(len(true_layers), len(layers))

check_layers([Convolution2D,  Activation, MaxPooling2D, Dropout, Convolution2D, Activation, MaxPooling2D, Flatten, Dense, Activation, Dense, Activation], model.layers)
assert model.layers[3].p == 0.5, 'Third layer should be a Dropout of 50%'

model.compile(loss='categorical_crossentropy', optimizer=Adam(), metrics=['accuracy'])
history = model.fit(X_train, y_train, batch_size=128, nb_epoch=5, verbose=1, validation_data=(X_val, y_val))

assert model.loss == 'categorical_crossentropy', 'Not using categorical_crossentropy loss function'
assert isinstance(model.optimizer, Adam), 'Not using adam optimizer'
assert len(history.history['acc']) >= 5, 'You\'re using {} epochs when you need to use 5 epochs.'.format(len(history.history['acc']))
assert history.history['acc'][-1] > 0.92, 'The training accuracy was: %.3f. It shoud be greater than 0.92' % history.history['acc'][-1]
assert history.history['val_acc'][-1] > 0.85, 'The validation accuracy is: %.3f. It shoud be greater than 0.85' % history.history['val_acc'][-1]
print('Training Network passed.')

############################
# Save the model and weights
############################
import json
model_json = model.to_json()
with open("./model.json", "w") as json_file:
    json.dump(model_json, json_file)
model.save_weights("./model.h5")
print("Saved model to disk")