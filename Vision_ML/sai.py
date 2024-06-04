import tensorflow as tf
import tensorflow_hub as hub
import numpy as np
import cv2
import matplotlib.pyplot as plt
import sys

# Load the correct object detection model from TensorFlow Hub
model_url = "https://tfhub.dev/tensorflow/ssd_mobilenet_v2/2"
detector = hub.load(model_url)

def process_image(img):
  # Perform inference on the image (replace with your existing code)
  image, boxes, classes, scores = perform_inference(img)

  # Draw bounding boxes on the image
  image_with_boxes = draw_boxes(image.copy(), boxes, classes, scores)

  # Create a dictionary to store processed data
  processed_data = {
      "image": image_with_boxes,
      "boxes": boxes,
      "classes": classes,
      "scores": scores
  }

  return processed_data

# Function to perform inference on a single image
def perform_inference(image_path):
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image at path '{image_path}' could not be loaded. Please check the file path.")
    
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # Convert image to tensor of type uint8
    image_tensor = tf.convert_to_tensor(image_rgb, dtype=tf.uint8)
    image_tensor = tf.expand_dims(image_tensor, axis=0)

    # Print the shape of the input tensor
    print(f"Input tensor shape: {image_tensor.shape}")
    
    # Perform detection
    detector_output = detector(image_tensor)

    # Print the keys of the detector output to see available tensors
    print("Detector output keys:", detector_output.keys())
    
    # Extract detection results
    detection_boxes = detector_output["detection_boxes"][0].numpy()
    detection_classes = detector_output["detection_classes"][0].numpy()
    detection_scores = detector_output["detection_scores"][0].numpy()
    
    return image_rgb, detection_boxes, detection_classes, detection_scores

# Function to draw bounding boxes on the image
def draw_boxes(image, boxes, classes, scores, min_score=0.5):
    for box, class_id, score in zip(boxes, classes, scores):
        if score >= min_score:
            ymin, xmin, ymax, xmax = box
            height, width, _ = image.shape
            start_point = (int(xmin * width), int(ymin * height))
            end_point = (int(xmax * width), int(ymax * height))
            color = (255, 0, 0)
            thickness = 2
            image = cv2.rectangle(image, start_point, end_point, color, thickness)
            label = f"Class {int(class_id)}: {score:.2f}"
            image = cv2.putText(image, label, start_point, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)
    return image

# Path to your image
image = img

try:
    # Perform inference
    image, boxes, classes, scores = perform_inference(image_path)

    # Draw boxes on the image
    image_with_boxes = draw_boxes(image, boxes, classes, scores)

    # Display the image
    plt.figure(figsize=(10, 10))
    plt.imshow(image_with_boxes)
    plt.axis('off')
    plt.show()
except FileNotFoundError as e:
    print(e)
except Exception as e:
    print(f"An error occurred: {e}")
