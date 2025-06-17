# gestos_robot/gestos_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2 as cv
from cvzone.HandTrackingModule import HandDetector
from cvzone.ClassificationModule import Classifier
import numpy as np
import math

class GestosNode(Node):
    def __init__(self):
        super().__init__('gestos_node')

        # 1️⃣ Crea el publisher
        self.publisher_ = self.create_publisher(String, '/ordenes', 10)

        # 2️⃣ Inicializa la cámara, detector y clasificador
        self.cap = cv.VideoCapture(0)
        self.detector = HandDetector(maxHands=1)
        self.classifier = Classifier("Model/keras_model.h5","Model/labels.txt")
        self.imgSize = 300
        self.offset = 5
        self.Labels = ["go_ahead", "stop" , "rotate"]
        self.Label_already_sent = None

        # 3️⃣ Crea un timer para procesar frames periódicamente (10 Hz)
        self.timer = self.create_timer(0.1, self.detectar_y_publicar)

    def detectar_y_publicar(self):
        success, img = self.cap.read()
        if not success:
            self.get_logger().warning('No se pudo capturar imagen.')
            return

        imgOutput = img.copy()
        hands, img = self.detector.findHands(img)

        if hands:
            hand = hands[0]
            x, y, w, h = hand['bbox']

            y1 = max(0, y)
            y2 = min(img.shape[0], y + h)
            x1 = max(0, x)
            x2 = min(img.shape[1], x + w)

            if y2 > y1 and x2 > x1:
                imgCrop = img[y1:y2, x1:x2]
                imgWhite = np.ones((self.imgSize, self.imgSize, 3), np.uint8) * 255

                aspectRatio = h / w
                if aspectRatio > 1:
                    k = self.imgSize / h
                    ratio = math.ceil(k * w)
                    imgResize = cv.resize(imgCrop, (ratio, self.imgSize))
                    Gap = math.ceil((self.imgSize - ratio) / 2)
                    imgWhite[:, Gap:ratio + Gap] = imgResize
                else:
                    k = self.imgSize / w
                    ratio = math.ceil(k * h)
                    imgResize = cv.resize(imgCrop, (self.imgSize, ratio))
                    Gap = math.ceil((self.imgSize - ratio) / 2)
                    imgWhite[Gap:ratio + Gap, :] = imgResize

                prediction, index = self.classifier.getPrediction(imgWhite)
                confianza = 0.95 if self.Labels[index] != "go_ahead" else 0.7

                if prediction[index] > confianza:
                    orden = self.Labels[index]
                    cv.putText(imgOutput, orden, (x, y - 20), cv.FONT_HERSHEY_COMPLEX, 2, (255, 0, 0))

                    if self.Label_already_sent != orden:
                        msg = String()
                        msg.data = orden
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'Orden publicada: {orden}')
                        self.Label_already_sent = orden

                cv.imshow("ImageWhite", imgWhite)
                cv.rectangle(imgOutput, (x - 20, y - 20), (x + w + 20, y + h + 20), (255, 0, 255), 4)

        cv.imshow("Image", imgOutput)

        if cv.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv.destroyAllWindows()
            rclpy.shutdown()
