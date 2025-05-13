
import os
from flask import Flask, request, jsonify
import cv2
import numpy as np
from ultralytics import YOLO
import time

app = Flask(__name__)

# Load model
model = YOLO(r"/Users/keysha/Downloads/img-rec/best.pt")
detected_images_dir = "/Users/keysha/Downloads/img_rec/processed_images"
output_dir = "/Users/keysha/Downloads/img_rec/output_images"
no_detection_dir = "/Users/keysha/Downloads/img_rec/no_detections"
os.makedirs(detected_images_dir, exist_ok=True)
os.makedirs(no_detection_dir, exist_ok=True)
saved_images = []
#model.to('cuda')  # Use GPU for inference

CLASS_NAMES = [
    'Bullseye', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
    'S', 'Left', 'Right', 'V', 'W', 'X', 'Y', 'Z',
    'Up Arrow', 'Down Arrow', 'Right Arrow', 'Left Arrow', 'Stop'
]

# Function to ensure directories exist before saving images
def ensure_directories():
    if not os.path.exists(detected_images_dir):
        os.makedirs(detected_images_dir)
        print(f"[DEBUG] Created directory: {detected_images_dir}")

    if not os.path.exists(no_detection_dir):
        os.makedirs(no_detection_dir)
        print(f"[DEBUG] Created directory: {no_detection_dir}")

@app.route('/process_frame', methods=['POST'])
def process_frame():
    global saved_images
    ensure_directories()
    file = request.files['file']
    file_bytes = np.frombuffer(file.read(),np.uint8)
    frame = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)

    # Original image dimensions (384x640)
    #original_height, original_width = 384, 640

        # Resize the image to 640x640 (or 416x416 if that's what your model was trained on)
        #frame_resized = cv2.resize(frame, (640, 640))  # Resize the frame to match model input

    # Predict with Ultralytics API (OBB task)
    results = model.predict(source=frame, conf=0.3, half=True)

        # Remapping class indices from the original to the new range
    class_index_remap = {
        0: 10, 1: 11, 2: 12, 3: 13, 4: 14, 5: 15, 6: 16, 7: 17, 8: 18, 9: 19,
        10: 20, 11: 21, 12: 22, 13: 23, 14: 24, 15: 25, 16: 26, 17: 27, 18: 28, 
        19: 29, 20: 30, 21: 31, 22: 32, 23: 33, 24: 34, 25: 35, 26: 36, 27: 37, 
        28: 38, 29: 39, 30: 40
    }

    highest_confidence = 0
    largest_detection = None
    detection_found = False  # Flag to track detections
    timestamp = int(time.time())
    filename = f"image_{timestamp}.jpg"
    output_filename = os.path.join(detected_images_dir, filename)        

    # Loop through results and find the highest confidence detection
    for result in results:
        # Check if any boxes were detected
        if result.boxes is not None and len(result.boxes) > 0:
            for box in result.boxes:
                # YOLOv8 'Boxes' object provides 'xywh' or 'xyxy' etc.
                x_center, y_center, width, height = box.xywh[0].tolist()
                confidence = float(box.conf.item())
                class_id = int(box.cls.item())
                
                # (Optional) Remap the class ID if needed
                remapped_class_id = class_index_remap.get(class_id, class_id)
                print(f"[DEBUG] Original class_id={class_id} => class_name='{CLASS_NAMES[remapped_class_id]}', conf={confidence:.2f}")

                # (Optional) Remap the class ID if needed
                remapped_class_id = class_index_remap.get(class_id, class_id)
                # Track the highest confidence detection
                if class_id!= 11 and confidence > highest_confidence:
                    highest_confidence = confidence
                    largest_detection = {
                        'bbox': [x_center, y_center, width, height],
                        'confidence': confidence,
                        #'symbol_map_id': remapped_class_id,
                        'class_name': CLASS_NAMES[class_id],
                        #'angle': angle,
                        'filename': filename
                    }
    cv2.imwrite(output_filename, frame)  # Save image before response

    # Plot only the largest detection using YOLO's built-in plot method
    if largest_detection:
        detection_result = {'largest_detection': largest_detection}
        for result in results:
        # Apply the plot method to visualize the largest detection
            result.plot()  # Automatically plots all the bounding boxes


        print(f"[DEBUG] Image with largest detection saved as {output_filename}")

        return jsonify(detection_result)
    else:
        return jsonify({"largest_detection": None, "message": "No detections found"}), 200

@app.route('/draw_bounding_boxes', methods=['POST'])
def draw_bounding_boxes():
    """Receives largest_detection data from RPi, draws bounding boxes, and saves the image."""

    data = request.json  # Expecting JSON data from RPi
    detection = data.get("largest_detection", {})
    print(detection)

    if not detection:
        return jsonify({"error": "No detection data provided"}), 400

    filename = detection.get('filename')  # Extract filename
    image_path = os.path.join(detected_images_dir, filename)

    if not os.path.exists(image_path):
        return jsonify({"error": f"Image {filename} not found"}), 404

    # Load the original image
    frame = cv2.imread(image_path)

    # For the bounding box, we stored xywh in largest_detection['bbox']
    x_center, y_center, width, height = detection['bbox']
    x_start = int(x_center - width / 2)
    y_start = int(y_center - height / 2)
    x_end = int(x_center + width / 2)
    y_end = int(y_center + height / 2)

    # Draw bounding box and label
    cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
    cv2.putText(frame, f"{detection['class_name']}: {detection['confidence']:.2f}",
                (x_start, y_start - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Save updated image
    output_filename = f"detection_{int(time.time())}.jpg"
    output_path = os.path.join(output_dir, output_filename)
    cv2.imwrite(output_path, frame)
    saved_images.append(output_path)

    return jsonify({
        "message": "Bounding box drawn",
        "image_path": output_path
    }), 200




@app.route('/stitch_images', methods=['GET'])
def stitch_images():
    global saved_images

    if len(saved_images) < 2:
        return jsonify({"message": "Not enough images to stitch"}), 400

    images = [cv2.imread(img) for img in saved_images if os.path.exists(img)]
    
    # Check if images were successfully loaded
    if len(images) < 2:
        return jsonify({"message": "Error loading images"}), 500

    # Stitch images side by side
    stitched_image = np.hstack(images)  # Use np.vstack(images) for vertical stacking

    # Save stitched image
    stitched_output = os.path.join(output_dir, "stitched_image.jpg")
    cv2.imwrite(stitched_output, stitched_image)

    return jsonify({"message": "Images stitched", "stitched_image": stitched_output}), 200


@app.route('/status', methods=['GET'])
def status():
    status_info = {
        "status": "running",
        "processed_images_count": len(saved_images),
        "timestamp": int(time.time())
    }
    return jsonify(status_info), 200


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=6000)
