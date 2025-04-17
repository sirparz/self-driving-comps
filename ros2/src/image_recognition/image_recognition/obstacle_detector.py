import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image #CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.subscription = self.create_subscription(
            Image,
            '/camera/color_image',  # Change to your image topic if different
            self.listener_callback,
            10
        )

        self.bridge = CvBridge()
        self.model = YOLO("yolov8m.pt")  # Load YOLOv8 model

        self.model.conf = 0.5  # Confidence threshold
        self.get_logger().info("YOLOv8 model loaded successfully.")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        results = self.model(cv_image, conf=0.5)  # Confidence threshold here

        # Process detections using new YOLOv8 format
        detections = results[0]
        names = self.model.names

        if detections.boxes is not None:
            for box in detections.boxes:
                cls_id = int(box.cls[0])
                cls_name = names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                if cls_name == 'traffic light':
                    # ROI check
                    height, width, _ = cv_image.shape
                    roi_top = 0
                    roi_bottom = int(height / 2)
                    roi_left = int(width / 3)
                    roi_right = int(2 * width / 3)

                    if x1 > roi_left and x2 < roi_right and y1 > roi_top and y2 < roi_bottom:
                        traffic_light_roi = cv_image[y1:y2, x1:x2]
                        if traffic_light_roi.size == 0:
                            continue

                        traffic_light_roi = cv2.resize(traffic_light_roi, None, fx=3.0, fy=3.0)
                        light_color = self.detect_traffic_light_color(traffic_light_roi)
                        label = f"Traffic Light ({light_color}, {conf:.2f})"

                        box_color = self.get_color_map(light_color)
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), box_color, 2)
                        cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                        self.get_logger().info(f"Traffic Light: {label}")

                elif cls_name == 'stop sign':
                    # Extract ROI
                    stop_roi = cv_image[y1:y2, x1:x2]
                    if stop_roi.size == 0:
                        continue

                    # Convert to grayscale and threshold to isolate shape
                    gray = cv2.cvtColor(stop_roi, cv2.COLOR_BGR2GRAY)
                    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                    _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

                    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    is_facing_camera = False
                    for cnt in contours:
                        approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
                        area = cv2.contourArea(cnt)
                        if len(approx) == 8 and area > 100:  # Approximate octagon
                            is_facing_camera = True
                            break

                    if is_facing_camera:
                        label = f"Stop Sign ({conf:.2f})"
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        self.get_logger().info(f"Stop Sign: {label}")
                    else:
                        self.get_logger().info("Rejected stop sign: not facing camera.")


        cv2.imshow('Object Detection', cv_image)
        cv2.waitKey(1)

    '''
    def detect_traffic_light_color(self, roi):
        roi = cv2.resize(roi, (60, 120), interpolation=cv2.INTER_CUBIC)
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        red_lower1 = np.array([0, 70, 50])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 70, 50])
        red_upper2 = np.array([180, 255, 255])

        green_lower = np.array([36, 50, 70])
        green_upper = np.array([89, 255, 255])

        red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        red_mask = red_mask1 + red_mask2
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        red_area = cv2.countNonZero(red_mask)
        green_area = cv2.countNonZero(green_mask)
        total_area = roi.shape[0] * roi.shape[1]

        if red_area > total_area * 0.02:
            return "Red"
        elif green_area > total_area * 0.02:
            return "Green"
        return "Unknown"

    '''
    
    def detect_traffic_light_color(self, roi):
        # Convert ROI to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Define red and green ranges
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([180, 255, 255])
        green_lower = np.array([40, 50, 50])
        green_upper = np.array([90, 255, 255])

        # Masks
        red_mask = cv2.inRange(hsv, red_lower1, red_upper1) + cv2.inRange(hsv, red_lower2, red_upper2)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # Optional: Apply blur to smooth noise
        red_mask = cv2.GaussianBlur(red_mask, (5, 5), 0)
        green_mask = cv2.GaussianBlur(green_mask, (5, 5), 0)

        # Find contours
        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        red_area = 0
        green_area = 0

        def compute_circular_area(contours):
            total_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                if circularity > 0.6 and area > 5:  # Circular and big enough
                    total_area += area
            return total_area

        red_area = compute_circular_area(contours_red)
        green_area = compute_circular_area(contours_green)

        threshold = 15  # Minimum area for valid detection

        # Decision logic based on area
        if red_area > green_area and red_area > threshold:
            return "Red"
        elif green_area > red_area and green_area > threshold:
            return "Green"
        else:
            return "Unknown"

    def get_color_map(self, color):
        """Map color names to BGR values for drawing on the image."""
        color_map = {
            'Red': (0, 0, 255),
            'Green': (0, 255, 0),
            'Unknown': (255, 255, 255),
        }
        return color_map.get(color, (255, 255, 255))

    def shutdown(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    try:
        rclpy.spin(object_detector)
    except KeyboardInterrupt:
        pass
    object_detector.shutdown()
    object_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
