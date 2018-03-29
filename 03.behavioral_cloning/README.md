
# Behavioral Cloning with Keras

## Network Structure

Our CNN is based on NVIDIA's CNN - Paper: [https://arxiv.org/pdf/1604.07316v1.pdf](https://arxiv.org/pdf/1604.07316v1.pdf). 
The network consists of 9 layers, including a normalization layer, 5 convolutional layers and 3 fully connected layers. The input image is split into YUV planes and passed to the network. The first layer of the network performs image normalization. The normalizer is hard-coded and is not adjusted in the learning process. Performing normalization in the network allows the normalization
scheme to be altered with the network architecture and to be accelerated via GPU processing. The convolutional layers were designed to perform feature extraction and were chosen empirically through a series of experiments that varied layer configurations. We use strided convolutions in the first three convolutional layers with a 2×2 stride and a 5×5 kernel and a non-strided convolution
with a 3×3 kernel size in the last two convolutional layers. We follow the five convolutional layers with three fully connected layers leading to an output control value which is the steering angle. The fully connected layers are designed to function as a
controller for steering.

As can be seen below, the layers includes a Lamda normalization layer, 5 convolutional layers and 3 fully connected layers with dropouts, ending up having 147,148 trainable parameters:

____________________________________________________________________________________________________
Layer (type)                     Output Shape          Param #     Connected to                     
======================================================
maxpooling2d_1 (MaxPooling2D)    (None, 33, 66, 3)     0           maxpooling2d_input_1[0][0]       
____________________________________________________________________________________________________
lambda_1 (Lambda)                (None, 33, 66, 3)     0           maxpooling2d_1[0][0]             
____________________________________________________________________________________________________
convolution2d_1 (Convolution2D)  (None, 9, 17, 5)      1805        lambda_1[0][0]                   
____________________________________________________________________________________________________
elu_1 (ELU)                      (None, 9, 17, 5)      0           convolution2d_1[0][0]            
____________________________________________________________________________________________________
convolution2d_2 (Convolution2D)  (None, 5, 9, 5)       4505        elu_1[0][0]                      
____________________________________________________________________________________________________
elu_2 (ELU)                      (None, 5, 9, 5)       0           convolution2d_2[0][0]            
____________________________________________________________________________________________________
convolution2d_3 (Convolution2D)  (None, 3, 5, 5)       6005        elu_2[0][0]                      
____________________________________________________________________________________________________
elu_3 (ELU)                      (None, 3, 5, 5)       0           convolution2d_3[0][0]            
____________________________________________________________________________________________________
convolution2d_4 (Convolution2D)  (None, 2, 3, 3)       2883        elu_3[0][0]                      
____________________________________________________________________________________________________
elu_4 (ELU)                      (None, 2, 3, 3)       0           convolution2d_4[0][0]            
____________________________________________________________________________________________________
convolution2d_5 (Convolution2D)  (None, 1, 2, 3)       1731        elu_4[0][0]                      
____________________________________________________________________________________________________
flatten_1 (Flatten)              (None, 6)             0           convolution2d_5[0][0]            
____________________________________________________________________________________________________
dropout_1 (Dropout)              (None, 6)             0           flatten_1[0][0]                  
____________________________________________________________________________________________________
elu_5 (ELU)                      (None, 6)             0           dropout_1[0][0]                  
____________________________________________________________________________________________________
dense_1 (Dense)                  (None, 1164)          8148        elu_5[0][0]                      
____________________________________________________________________________________________________
dropout_2 (Dropout)              (None, 1164)          0           dense_1[0][0]                    
____________________________________________________________________________________________________
elu_6 (ELU)                      (None, 1164)          0           dropout_2[0][0]                  
____________________________________________________________________________________________________
dense_2 (Dense)                  (None, 100)           116500      elu_6[0][0]                      
____________________________________________________________________________________________________
dropout_3 (Dropout)              (None, 100)           0           dense_2[0][0]                    
____________________________________________________________________________________________________
elu_7 (ELU)                      (None, 100)           0           dropout_3[0][0]                  
____________________________________________________________________________________________________
dense_3 (Dense)                  (None, 50)            5050        elu_7[0][0]                      
____________________________________________________________________________________________________
dropout_4 (Dropout)              (None, 50)            0           dense_3[0][0]                    
____________________________________________________________________________________________________
elu_8 (ELU)                      (None, 50)            0           dropout_4[0][0]                  
____________________________________________________________________________________________________
dense_4 (Dense)                  (None, 10)            510         elu_8[0][0]                      
____________________________________________________________________________________________________
dropout_5 (Dropout)              (None, 10)            0           dense_4[0][0]                    
____________________________________________________________________________________________________
elu_9 (ELU)                      (None, 10)            0           dropout_5[0][0]                  
____________________________________________________________________________________________________
dense_5 (Dense)                  (None, 1)             11          elu_9[0][0]                      
======================================================


Pre-processing steps are in the following:
1. Re-sizing the image from from 320x160 to 200x66.
2. Convert the 200x66 image from RGB to YUV.
3. Crop the first 22 rows from the top so the image is now 200x44 and then resize it back to 200x66.
And these steps are also included in the new drive.py module.

## Optimizer
We used the Keras builtin support for the [Adam](http://sebastianruder.com/optimizing-gradient-descent/index.html#adam) optimizer. The Adam optimizer, as explained in project 2, is Kingma and Ba's modified version of the Stochastic Gradient Descent that allows the use of larger step sizes without fine tuning. It uses cross entropy calculations to minimize loss and use gradient descent, an iterative optimization technique and algorithm to achieve this goal. We decided to restrict the learning rates to 0.00001, making the model never converge, never over-fit.

## Training dataset
The training data were collected using the simulator for track 1 for 10 times. First, we drove and recorded normal laps around the track in the center for 10 times, and we found that the CNN had a tendency of moving to the left, so we ran Track1 again 10 times in the reverse (clockwise) direction. ~~We constantly wander off to the side of the road and then wandered back to the middle: to the left for 2 times and to the right for 2 times.~~ Instead as suggested, we did the following: Stop the recording, drive to the shoulder of the road, stop the car before it crosses the lane line. Turn the car wheels to the middle of the road, and then start recording, only then start driving; repeat that process in any place the car cross the lane lines or drive off the road. 

The training data collected include two sets of files. A CSV that contains the file path to the current center, right and left camara images, as well as, the current throttle, brake, steering angle, and speed. The trainer uses the model.fit\_generator function in Keras to bring in one image at a time to feed the GTX 1070 pascal GPU that we use for the training and validation. 

## Train/Validation Split

We split the training data into a training and validation set, and measured the [validation accuracy](https://keras.io/models/sequential/) of the network after two training epochs. We [Use the `train_test_split()` method](http://scikit-learn.org/stable/modules/generated/sklearn.model_selection.train_test_split.html) from scikit-learn.

## Train the netwrok
We trained the network for 1000 epochs. Use the adam optimizer, with categorical\_crossentropy loss. Keras's .fit() method returns a History.history object, which the tests below use. Save that to a variable named history.

1346/1347 [============================>.] - ETA: 0s - loss: 0.0396 - acc: 0.7779Epoch 00495: val_acc did not improve
1348/1347 [==============================] - 8s - loss: 0.0395 - acc: 0.7782 - val_loss: 0.0239 - val_acc: 0.7463
Epoch 497/500
1347/1347 [============================>.] - ETA: 0s - loss: 0.0357 - acc: 0.7780Epoch 00496: val_acc did not improve
1348/1347 [==============================] - 8s - loss: 0.0357 - acc: 0.7782 - val_loss: 0.0735 - val_acc: 0.7687
Epoch 498/500
1346/1347 [============================>.] - ETA: 0s - loss: 0.0455 - acc: 0.7764Epoch 00497: val_acc did not improve
1348/1347 [==============================] - 8s - loss: 0.0455 - acc: 0.7760 - val_loss: 0.0646 - val_acc: 0.7090
Epoch 499/500
1346/1347 [============================>.] - ETA: 0s - loss: 0.0268 - acc: 0.7964Epoch 00498: val_acc did not improve
1348/1347 [==============================] - 8s - loss: 0.0268 - acc: 0.7960 - val_loss: 0.0300 - val_acc: 0.7612
Epoch 500/500
1346/1347 [============================>.] - ETA: 0s - loss: 0.0455 - acc: 0.7771Epoch 00499: val_acc did not improve
1348/1347 [==============================] - 8s - loss: 0.0455 - acc: 0.7767 - val_loss: 0.0499 - val_acc: 0.7836

I found the optimal number of Epochs is 95, which provides the maximum performance (minimum validation loss) on the validation set. That can save processing time and will promise optimal network for the following analysis.