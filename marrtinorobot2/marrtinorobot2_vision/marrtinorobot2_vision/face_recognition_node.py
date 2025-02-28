import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')

        # Carica il modello di deep learning per il riconoscimento facciale
        self.net = cv2.dnn.readNetFromCaffe(
            'deploy.prototxt',
            'res10_300x300_ssd_iter_140000.caffemodel'
        )

        # Inizializza il CvBridge
        self.bridge = CvBridge()

        # Sottoscrivi al topic della fotocamera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Converti il messaggio ROS in un'immagine OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Prepara l'immagine per il modello di deep learning
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))

        self.net.setInput(blob)
        detections = self.net.forward()

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # Disegna un rettangolo intorno al volto rilevato
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)

        cv2.imshow('Video', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    rclpy.spin(node)

    # Rilascio delle risorse
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
