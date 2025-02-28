import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge


class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, current_value):
        error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
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

        # Publisher per il numero di volti rilevati
        self.face_count_publisher = self.create_publisher(Int32, '/nroface', 10)

        # PID controller per il pan e tilt
        self.pan_pid = PIDController(kp=0.1, ki=0.0, kd=0.1)
        self.tilt_pid = PIDController(kp=0.1, ki=0.0, kd=0.1)

        # Parametri di default per il servo pan/tilt
        self.servomaxx = 1023   # Massima rotazione servo orizzontale (x)
        self.servomaxy = 1023   # Massima rotazione servo verticale (y)
        self.servomin = 0       # Minima rotazione servo
        self.center_pos_x = 512  # Posizione centrale servo orizzontale (x)
        self.center_pos_y = 512  # Posizione centrale servo verticale (y)

        self.current_pos_x = float(self.center_pos_x)
        self.current_pos_y = float(self.center_pos_y)

        # Calcolo dei margini centrali per il tracciamento
        self.screenmaxx = 640  # Risoluzione massima dello schermo (x)
        self.screenmaxy = 480  # Risoluzione massima dello schermo (y)

        # Imposta la posizione iniziale centrale
        self.initial_pose_x = Float64()
        self.initial_pose_x.data = float(self.center_pos_x)

        self.initial_pose_y = Float64()
        self.initial_pose_y.data = float(self.center_pos_y)

        self.dynamixel_control.publish(self.initial_pose_x)
        self.dynamixel_control_tilt.publish(self.initial_pose_y)

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

        face_count = 0  # Numero di volti rilevati

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5:
                face_count += 1  # Incrementa il numero di volti rilevati

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

                # Stampa di debug
                self.get_logger().info(f'Offset X: {offset_x}, Offset Y: {offset_y}')

                # Chiama la funzione di tracciamento del volto con gli offset calcolati
                self.track_face(offset_x, offset_y)

        # Pubblica il numero di volti rilevati
        face_count_msg = Int32()
        face_count_msg.data = face_count
        self.face_count_publisher.publish(face_count_msg)
        if (face_count > 1 ):
            self.get_logger().info(f'Numero di volti rilevati: {face_count}')

        # Mostra l'immagine con il volto rilevato
        cv2.imshow('Face Recognition', frame)
        cv2.waitKey(1)

    def track_face(self, x, y):
        # Usa il PID controller per regolare la velocità di spostamento
        pan_output = self.pan_pid.compute(x)
        tilt_output = self.tilt_pid.compute(y)

        # Stampa di debug per vedere il risultato del PID
        self.get_logger().info(f'Pan Output: {pan_output}, Tilt Output: {tilt_output}')

        # Aggiorna la posizione corrente del servo per il pan
        self.current_pos_x += pan_output
        self.current_pos_x = max(min(self.current_pos_x, self.servomaxx), self.servomin)
        current_pose_x = Float64()
        current_pose_x.data = self.current_pos_x
        self.dynamixel_control.publish(current_pose_x)

        # Aggiorna la posizione corrente del servo per il tilt
        self.current_pos_y += tilt_output
        self.current_pos_y = max(min(self.current_pos_y, self.servomaxy), self.servomin)
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
