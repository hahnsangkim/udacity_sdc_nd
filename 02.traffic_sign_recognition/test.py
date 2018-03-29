#########################
# Test the Network
#########################
def normalize_grayscale(image_data):
    a = -0.5
    b = 0.5
    grayscale_min = 0
    grayscale_max = 255
    return a + ( ( (image_data - grayscale_min)*(b - a) )/( grayscale_max - grayscale_min ) )

import pickle
import json
from sklearn.preprocessing import LabelBinarizer
from keras.models import model_from_json

with open('test.p', 'rb') as f:
    data_test = pickle.load(f)

X_test = data_test['features']
y_test = data_test['labels']
X_normalized_test = normalize_grayscale(X_test)
label_binarizer = LabelBinarizer()
y_one_hot_test = label_binarizer.fit_transform(y_test)

with open('model.json', 'r') as jfile:
    model = model_from_json(json.load(jfile))
model.compile('adam', 'categorical_crossentropy', metrics = ['accuracy'])
model.load_weights('model.h5')

score = model.evaluate(X_normalized_test, y_one_hot_test, verbose=1)
for metric_i in range(len(model.metrics_names)):
    metric_name = model.metrics_names[metric_i]
    metric_value = score[metric_i]
    print('{}: {}'.format(metric_name, metric_value))
