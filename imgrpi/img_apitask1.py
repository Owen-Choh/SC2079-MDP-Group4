from flask import Flask, request, jsonify
import cv2
import numpy as np
from ultralytics import YOLO
import time
import os

app = Flask(__name__)

# Load model (.onnx or .pt)
model = YOLO(r"C:/Users/keysh/Downloads/img_rec/runs/obb/yolov8s/weights/best.pt")  # or 'best.pt'
model.to("cuda")
BASE_DIR = r"C:/Users/keysh/Downloads/img_rec"

detected_images_dir = "processed_images"
no_detection_dir = "no_detections"

detected_images_dir = os.path.join(BASE_DIR, "processed_images")
no_detection_dir = os.path.join(BASE_DIR, "no_detections")

os.makedirs(detected_images_dir, exist_ok=True)
os.makedirs(no_detection_dir, exist_ok=True)
saved_images = []

CLASS_NAMES = [
    'Bullseye', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
    'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
    'Up Arrow', 'Down Arrow', 'Right Arrow', 'Left Arrow', 'Stop'
]

@app.route('/process_frame', methods=['POST'])
def process_frame():
    print("process_frame called!") 
    global saved_images
    file = request.files['file']
    file_bytes = np.frombuffer(file.read(),np.uint8)
    frame = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)

    # Original image dimensions (384x640)
    original_height, original_width = 384, 640

    # Resize the image to 640x640 (or 416x416 if that's what your model was trained on)
    # Resize the image to 640x640 (or 416x416 if that's what your model was trained on)
    frame_resized = cv2.resize(frame, (640, 640))  # Resize the frame to match model input

    # Predict with Ultralytics API (OBB task)
    results = model.predict(source=frame_resized, task='obb', conf=0.3, half=True)

    # Remapping class indices from the original to the new range
    class_index_remap = {
        0: 10, 1: 11, 2: 12, 3: 13, 4: 14, 5: 15, 6: 16, 7: 17, 8: 18, 9: 19,
        10: 20, 11: 21, 12: 22, 13: 23, 14: 24, 15: 25, 16: 26, 17: 27, 18: 28, 
        19: 29, 20: 30, 21: 31, 22: 32, 23: 33, 24: 34, 25: 35, 26: 36, 27: 37, 
        28: 38, 29: 39, 30: 40
    }

    highest_confidence = 0
    largest_detection = None

    # Loop through results and find the highest confidence detection
    for result in results:
        for box in result.obb:
            # Convert tensors to native Python types
            x_center, y_center, width, height, angle = box.xywhr[0].tolist()  # Convert tensor to list
            class_id = int(box.cls.item())  # Convert tensor to int
            confidence = float(box.conf.item())  # Convert tensor to float
            
            extended_height = height * 2
            x_start = int(x_center - width / 2)
            y_start = int(y_center - extended_height / 2)
            x_end = int(x_center + width / 2)
            y_end = int(y_center + extended_height / 2)


            # Remap the class ID based on the dictionary
            remapped_class_id = class_index_remap.get(class_id, class_id)  # Use original class_id if not found in the map
            cv2.rectangle(frame_resized, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)  # Green box with 2px thickness
            cv2.putText(frame_resized, f"{CLASS_NAMES[class_id]}: {confidence:.2f}", (x_start, y_start - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


            # Track the highest confidence detection
            if confidence > highest_confidence:
                highest_confidence = confidence
                largest_detection = {
                    'bbox': [x_center, y_center, width, height],
                    'confidence': confidence,
                    'symbol_map_id': remapped_class_id,
                    'class_name': CLASS_NAMES[class_id],
                    'angle': angle
                }
    if largest_detection:
        detection_result = {'largest_detection': largest_detection}
        for result in results:
            result.plot()  # Plots detections

        timestamp = int(time.time())


        output_filename = os.path.join(detected_images_dir, f"detection_{timestamp}.jpg")

        cv2.imwrite(output_filename, frame_resized)  # ✅ Always save image
        print(f"[DEBUG] Image saved as {output_filename}")

        return jsonify(detection_result)
    else:
        # ✅ Save image even if no detections
        timestamp = int(time.time())
        output_filename = os.path.join(no_detection_dir, f"no_detection_{timestamp}.jpg")
        cv2.imwrite(output_filename, frame_resized)
        
        print(f"[DEBUG] No detections. Image saved as {output_filename}")
        return jsonify({"largest_detection": None, "message": "No detections found", "image_saved": output_filename}), 200


@app.route('/stitch_images', methods=['GET'])
def stitch_images():
    """
    Fetches images from 'processed_images' folder and arranges them in a collage format.
    """
    processed_images = sorted(
        [os.path.join(detected_images_dir, f) for f in os.listdir(detected_images_dir) if f.endswith((".jpg", ".png"))]
    )

    if len(processed_images) < 1:
        return jsonify({"message": "Not enough images in processed_images folder to create a collage"}), 400

    images = [cv2.imread(img) for img in processed_images if os.path.exists(img)]
    images = [img for img in images if img is not None]  # Remove failed loads

    if len(images) < 1:
        return jsonify({"message": "Error loading images"}), 500

    # Resize images to a fixed size (optional)
    collage_width = 1280  # Set collage width (adjust as needed)
    num_images = len(images)
    grid_size = int(np.ceil(np.sqrt(num_images)))  # Calculate grid dimensions (NxN)
    
    # Set target size for each image (based on grid size)
    target_size = collage_width // grid_size
    images = [cv2.resize(img, (target_size, target_size)) for img in images]

    # Arrange images into a grid
    rows = []
    for i in range(0, num_images, grid_size):
        row_images = images[i:i+grid_size]
        while len(row_images) < grid_size:  # Fill empty slots with blank images
            row_images.append(np.zeros_like(row_images[0]))
        rows.append(np.hstack(row_images))  # Stack horizontally

    # Stack all rows vertically
    collage = np.vstack(rows)

    # Save the collage
    timestamp = int(time.time())
    collage_output = os.path.join(BASE_DIR, f"collage_output_{timestamp}.jpg")
    cv2.imwrite(collage_output, collage)

    return jsonify({
        "message": "Collage created successfully",
        "collage_image": collage_output
    }), 200


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=6000)