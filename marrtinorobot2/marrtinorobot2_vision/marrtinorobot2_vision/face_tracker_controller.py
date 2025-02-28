import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        # Calcolo PID
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output


class FaceRecognitionAndTrackingNode(Node):
    def __init__(self):
        super().__init__('face_recognition_and_tracking_node')

        # Carica il modello di deep learning per il riconoscimento facciale
        self.net = cv2.dnn.readNetFromCaffe(
            'deploy.prototxt',
            'res10_300x300_ssd_iter_140000.caffemodel'
        )

        # Inizializza il CvBridge
        self.bridge = CvBridge()

        # Sottoscrizione al topic della fotocamera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher per il controllo del Dynamixel
        self.dynamixel_control = self.create_publisher(Float64, '/pan_controller/command', 10)
        self.dynamixel_control_tilt = self.create_publisher(Float64, '/tilt_controller/command', 10)

        # Publisher per il numero di volti
        self.face_count_pub = self.create_publisher(Float64, '/nroface', 10)

        # Parametri di default per il servo pan/tilt
        self.servomaxx = 1023   # Massima rotazione servo orizzontale (x)
        self.servomaxy = 1023   # Massima rotazione servo verticale (y)
        self.servomin = 0       # Minima rotazione servo
        self.center_pos_x = 512  # Posizione centrale servo orizzontale (x)
        self.center_pos_y = 512  # Posizione centrale servo verticale (y)
        self.current_pos_x = float(self.center_pos_x)
        self.current_pos_y = float(self.center_pos_y)

        # PID controller per pan e tilt
        self.pid_x = PIDController(0.05, 0.001, 0.01)  # Regola questi valori per rallentare il movimento
        self.pid_y = PIDController(0.05, 0.001, 0.01)

        # Calcolo dei margini centrali per il tracciamento
        self.screenmaxx = 640  # Risoluzione massima dello schermo (x)
        self.screenmaxy = 480  # Risoluzione massima dello schermo (y)
        self.center_offset = 100
        self.center_offsety = 60
        self.center_left = (self.screenmaxx / 2) - self.center_offset
        self.center_right = (self.screenmaxx / 2) + self.center_offset
        self.center_up = (self.screenmaxy / 2) - self.center_offsety
        self.center_down = (self.screenmaxy / 2) + self.center_offsety

        # Imposta la posizione iniziale centrale
        self.initial_pose_x = Float64()
        self.initial_pose_x.data = float(self.center_pos_x)
        self.initial_pose_y = Float64()
        self.initial_pose_y.data = float(self.center_pos_y-100)
        self.dynamixel_control.publish(self.initial_pose_x)
        self.dynamixel_control_tilt.publish(self.initial_pose_y)

    def image_callback(self, msg):
        try:
            # Converti il messaggio ROS in un'immagine OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Capovolgi l'immagine orizzontalmente
            frame = cv2.flip(frame, 1)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Prepara l'immagine per il modello di deep learning
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))

        self.net.setInput(blob)
        detections = self.net.forward()

        face_count = 0
        face_found = False

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5:
                face_count += 1  # Incrementa il contatore dei volti
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # Disegna un rettangolo intorno al volto rilevato
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)

                # Calcola il centro del volto rilevato
                face_center_x = (startX + endX) // 2
                face_center_y = (startY + endY) // 2

                # Calcola l'offset rispetto al centro dell'immagine
                offset_x = face_center_x - (w // 2)
                offset_y = face_center_y - (h // 2)

                # Chiama la funzione di tracciamento del volto con gli offset calcolati
                self.track_face(offset_x, offset_y)

                face_found = True  # Indica che un volto è stato trovato

        # Pubblica il numero di volti rilevati
        face_count_msg = Float64()
        face_count_msg.data = float(face_count)
        self.face_count_pub.publish(face_count_msg)

        # Mostra l'immagine con il volto rilevato
        cv2.imshow('Face Recognition', frame)
        cv2.waitKey(1)

    def track_face(self, x, y):
        # Calcolo PID per X e Y
        control_x = self.pid_x.compute(x)
        control_y = self.pid_y.compute(y)

        # Controllo asse X (pan)
        self.current_pos_x += control_x
        if self.current_pos_x <= self.servomaxx and self.current_pos_x >= self.servomin:
            current_pose_x = Float64()
            current_pose_x.data = self.current_pos_x
            self.dynamixel_control.publish(current_pose_x)

        # Controllo asse Y (tilt)
        self.current_pos_y += control_y
        if self.current_pos_y <= self.servomaxy and self.current_pos_y >= self.servomin:
            current_pose_y = Float64()
            current_pose_y.data = self.current_pos_y
            self.dynamixel_control_tilt.publish(current_pose_y)


def main(args=None):
    rclpy.init(args=args)
    face_recognition_and_tracking_node = FaceRecognitionAndTrackingNode()
    rclpy.spin(face_recognition_and_tracking_node)
    face_recognition_and_tracking_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
