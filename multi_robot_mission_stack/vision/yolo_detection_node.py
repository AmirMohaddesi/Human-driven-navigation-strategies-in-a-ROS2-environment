import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2
import numpy as np

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('weights_path', '')
        self.declare_parameter('cfg_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_class_id', 0)  # COCO: person
        self.declare_parameter('display', False)

        image_topic = self.get_parameter('image_topic').value
        weights_path = self.get_parameter('weights_path').value
        cfg_path = self.get_parameter('cfg_path').value

        self._confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self._target_class_id = int(self.get_parameter('target_class_id').value)
        self._display = bool(self.get_parameter('display').value)

        if not weights_path or not cfg_path:
            # Repo does not ship YOLOv3 weights by default; try a conventional
            # package-local location but allow users to override.
            pkg_share = get_package_share_directory('multi_robot_mission_stack')
            weights_path = weights_path or os.path.join(pkg_share, 'vision', 'yolov3.weights')
            cfg_path = cfg_path or os.path.join(pkg_share, 'vision', 'yolov3.cfg')

        self._net = None
        self._output_layers = None
        self._warned_no_model = False

        if os.path.isfile(weights_path) and os.path.isfile(cfg_path):
            self.get_logger().info(f'Loading YOLO model from:\n- {weights_path}\n- {cfg_path}')
            self._net = cv2.dnn.readNet(weights_path, cfg_path)
            layer_names = self._net.getLayerNames()
            unconnected = self._net.getUnconnectedOutLayers()
            unconnected = np.array(unconnected).reshape(-1).astype(int)
            self._output_layers = [layer_names[i - 1] for i in unconnected]
        else:
            self.get_logger().warning(
                'YOLO weights/config not found. '
                'Detections are disabled. '
                'Set `weights_path` and `cfg_path` parameters to enable YOLO inference.'
            )

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        if self._net is None or self._output_layers is None:
            if not self._warned_no_model:
                self.get_logger().warning('Skipping YOLO inference: model is not loaded.')
                self._warned_no_model = True
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        height, width = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, scalefactor=0.00392, size=(416, 416), mean=(0, 0, 0), swapRB=True, crop=False)
        self._net.setInput(blob)
        layer_outputs = self._net.forward(self._output_layers)

        for output in layer_outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = float(scores[class_id])
                if confidence > self._confidence_threshold and class_id == self._target_class_id:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if self._display:
            cv2.imshow("YOLO Detection", frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()