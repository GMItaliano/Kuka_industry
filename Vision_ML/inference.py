import sys
import tensorflow as tf
import numpy as np
import cv2
import matplotlib.pyplot as plt

# Constants for image resizing
img_height, img_width = 224, 224

# Load the model
model = tf.keras.models.load_model('can_classifier_model.h5')

# Function to preprocess the image
def preprocess_image(image_path):
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image at path '{image_path}' could not be loaded. Please check the file path.")
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_resized = cv2.resize(image_rgb, (img_height, img_width))
    image_array = np.expand_dims(image_resized, axis=0)
    return image_array / 255.0

# Path to a new image for classification
new_image_path = 'C:/Users/gugal/Desktop/Github/Kuka_industry/Vision_ML/received_image.png'

image_array = preprocess_image(new_image_path)

# Predict the class of the image
predictions = model.predict(image_array)
predicted_class = np.argmax(predictions)
class_names = ['CanTypeA', 'CanTypeB']  # Replace with actual class names used during training
predicted_class_name = class_names[predicted_class]

print(f"Predicted class: {predicted_class_name}")

# Display the image with the prediction
plt.figure(figsize=(6, 6))
plt.imshow(image_array[0])
plt.title(f"Predicted class: {predicted_class_name}")
plt.axis('off')
plt.show()