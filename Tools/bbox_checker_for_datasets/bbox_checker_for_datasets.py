import cv2
import os
import random

# Directories and file paths
image_dir = './train'
class_filename = './classnames.txt'

# Read class names
with open(class_filename, "r") as file:
    class_names = file.read().splitlines()

# Function to process and draw bounding boxes on an image
def process_image(image_path, bbox_data_path):
    image = cv2.imread(image_path)
    image_height, image_width, _ = image.shape
    bboxes = []

    with open(bbox_data_path, "r") as file:
        for line in file:
            parts = line.strip().split()
            class_id = int(parts[0])
            center_x = float(parts[1]) * image_width
            center_y = float(parts[2]) * image_height
            width = float(parts[3]) * image_width
            height = float(parts[4]) * image_height
            x1 = int(center_x - width / 2)
            y1 = int(center_y - height / 2)
            x2 = int(center_x + width / 2)
            y2 = int(center_y + height / 2)
            bboxes.append((class_id, x1, y1, x2, y2))

    for bbox in bboxes:
        class_id, x1, y1, x2, y2 = bbox
        label = class_names[class_id]
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    output_image_path = image_path.replace(image_dir, image_dir + '_bbox_check_result')
    os.makedirs(os.path.dirname(output_image_path), exist_ok=True)
    cv2.imwrite(output_image_path, image)

# Get list of image files
image_files = [f for f in os.listdir(image_dir) if f.endswith('.png') or f.endswith('.jpg')]
selected_images = random.sample(image_files, 100)

# Process selected images
for image_file in selected_images:
    image_path = os.path.join(image_dir, image_file)
    bbox_data_path = image_path.replace('.png', '.txt').replace('.jpg', '.txt')
    if os.path.exists(bbox_data_path):
        process_image(image_path, bbox_data_path)