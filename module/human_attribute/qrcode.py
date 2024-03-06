import cv2
import numpy as np
from pyzbar.pyzbar import decode
from hsr_agent.agent import Agent
import rospy


def decoder_loop(agent):
    while True:
        frame = agent.rgb_img
        gray_img = cv2.cvtColor(frame, 0)
        barcode = decode(gray_img)

        if len(barcode) != 0:
            for obj in barcode:
                points = obj.polygon
                (x, y, w, h) = obj.rect
                pts = np.array(points, np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 255, 0), 3)

                barcodeData = obj.data.decode("utf-8")
                barcodeType = obj.type
                string = "Data " + str(barcodeData) + " | Type " + str(barcodeType)

                cv2.putText(frame, string, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                print("Barcode: " + barcodeData + " | Type: " + barcodeType)

                cv2.destroyAllWindows()

                return barcodeData

        cv2.imshow('Image', frame)
        code = cv2.waitKey(10)
        if code == ord('q'):
            break


def decoder(image):
    gray_img = cv2.cvtColor(image, 0)
    barcode = decode(gray_img)

    print(len(barcode))
    for obj in barcode:
        points = obj.polygon
        (x, y, w, h) = obj.rect
        pts = np.array(points, np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(image, [pts], True, (0, 255, 0), 3)

        barcodeData = obj.data.decode("utf-8")
        barcodeType = obj.type
        string = "Data " + str(barcodeData) + " | Type " + str(barcodeType)

        cv2.putText(frame, string, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        print("Barcode: " + barcodeData + " | Type: " + barcodeType)
        return barcodeData


if __name__ == "__main__":
    rospy.init_node('qr_code', disable_signals=True)
    cap = Agent()
    while True:
        frame = cap.rgb_img
        decoder(frame)
        cv2.imshow('Image', frame)
        code = cv2.waitKey(10)
        if code == ord('q'):
            break