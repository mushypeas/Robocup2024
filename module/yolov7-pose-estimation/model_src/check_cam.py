import cv2

# Try to open the device using OpenNI2
cap = cv2.VideoCapture(cv2.CAP_OPENNI2)

if not cap.isOpened():
    print("Failed to open OpenNI2 device")
else:
    print("Successfully opened OpenNI2 device")
    while True:
        # Capture depth map
        if cap.grab():
            ret, depth_map = cap.retrieve(cv2.CAP_OPENNI_DEPTH_MAP)
            if ret:
                cv2.imshow("Depth Map", depth_map / 4500.0)
            
            ret, rgb_image = cap.retrieve(cv2.CAP_OPENNI_BGR_IMAGE)
            if ret:
                cv2.imshow("RGB Image", rgb_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
