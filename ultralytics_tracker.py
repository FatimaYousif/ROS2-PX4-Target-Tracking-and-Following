#!/usr/bin/env python3

import cv_bridge
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
# from ultralytics_ros.msg import YoloResult  # Remove 


class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker_node")
        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("input_topic", "camera")
        self.declare_parameter("result_topic", "detection_result")
        self.declare_parameter("result_image_topic", "yolo_image")
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("max_det", 300)
        self.declare_parameter("classes", [0])
        self.declare_parameter("tracker", "bytetrack.yaml")
        # self.declare_parameter("device", "cpu")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("result_conf", True)
        self.declare_parameter("result_line_width", 1)
        self.declare_parameter("result_font_size", 1)
        self.declare_parameter("result_font", "Arial.ttf")
        self.declare_parameter("result_labels", True)
        self.declare_parameter("result_boxes", True)

        path = get_package_share_directory("ultralytics_ros")
        yolo_model = self.get_parameter("yolo_model").get_parameter_value().string_value
        self.model = YOLO(f"{path}/models/{yolo_model}")
        self.model.fuse()

        self.bridge = cv_bridge.CvBridge()
        # self.use_segmentation = yolo_model.endswith("-seg.pt")
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        result_topic = (
            self.get_parameter("result_topic").get_parameter_value().string_value
        )
        result_image_topic = (
            self.get_parameter("result_image_topic").get_parameter_value().string_value
        )
        self.create_subscription(Image, input_topic, self.image_callback, 1)
        # Change this line to publish Detection2DArray instead of YoloResult
        self.results_pub = self.create_publisher(Detection2DArray, result_topic, 1)
        self.result_image_pub = self.create_publisher(Image, result_image_topic, 1)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        conf_thres = self.get_parameter("conf_thres").get_parameter_value().double_value
        iou_thres = self.get_parameter("iou_thres").get_parameter_value().double_value
        max_det = self.get_parameter("max_det").get_parameter_value().integer_value
        classes = (
            self.get_parameter("classes").get_parameter_value().integer_array_value
        )
        tracker = self.get_parameter("tracker").get_parameter_value().string_value
        device = self.get_parameter("device").get_parameter_value().string_value or None
        
        # device = self.get_parameter("device").get_parameter_value().string_value
        # print(".......................", device)
        # self.get_logger().info(f"+++..............Using device: {device}")
        results = self.model.track(
            source=cv_image,
            conf=conf_thres,
            iou=iou_thres,
            max_det=max_det,
            classes=classes,
            tracker=tracker,
            device=device,
            verbose=False,
            # retina_masks=True,
        )

        if results is not None:
            # Remove YoloResult message creation
            detections_msg = self.create_detections_array(results)
            detections_msg.header = msg.header
            
            yolo_result_image_msg = Image()
            yolo_result_image_msg.header = msg.header
            yolo_result_image_msg = self.create_result_image(results)
            
            # Publish the Detection2DArray directly
            self.results_pub.publish(detections_msg)
            self.result_image_pub.publish(yolo_result_image_msg)

    def create_detections_array(self, results):
        # print("checking.----")
        detections_msg = Detection2DArray()
        if not results or len(results[0].boxes) == 0:
            return detections_msg
            
        bounding_box = results[0].boxes.xywh
        classes = results[0].boxes.cls
        confidence_score = results[0].boxes.conf
        ids = results[0].boxes.id  # This gets the tracking IDs
        
        for i, (bbox, cls, conf) in enumerate(zip(bounding_box, classes, confidence_score)):
            detection = Detection2D()
            detection.bbox.center.position.x = float(bbox[0])
            detection.bbox.center.position.y = float(bbox[1])
            detection.bbox.size_x = float(bbox[2])
            detection.bbox.size_y = float(bbox[3])
            
            # Set the tracking ID if available
            if ids is not None and i < len(ids):
                detection.id = str(int(ids[i]))  # Convert to string
            
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = results[0].names.get(int(cls))
            hypothesis.hypothesis.score = float(conf)
            detection.results.append(hypothesis)
            detections_msg.detections.append(detection)
        
        return detections_msg

    def create_result_image(self, results):
        result_conf = self.get_parameter("result_conf").get_parameter_value().bool_value
        result_line_width = (
            self.get_parameter("result_line_width").get_parameter_value().integer_value
        )
        result_font_size = (
            self.get_parameter("result_font_size").get_parameter_value().integer_value
        )
        result_font = (
            self.get_parameter("result_font").get_parameter_value().string_value
        )
        result_labels = (
            self.get_parameter("result_labels").get_parameter_value().bool_value
        )
        result_boxes = (
            self.get_parameter("result_boxes").get_parameter_value().bool_value
        )
        plotted_image = results[0].plot(
            conf=result_conf,
            line_width=result_line_width,
            font_size=result_font_size,
            font=result_font,
            labels=result_labels,
            boxes=result_boxes,
        )
        result_image_msg = self.bridge.cv2_to_imgmsg(plotted_image, encoding="bgr8")
        return result_image_msg


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    
    rclpy.spin(node)


if __name__ == "__main__":
    main()
