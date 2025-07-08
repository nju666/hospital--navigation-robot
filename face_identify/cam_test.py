import cv2

def test_camera(device_id=0):
    cap = cv2.VideoCapture(device_id)

    if not cap.isOpened():
        print(f"无法打开摄像头 /dev/video{device_id}")
        return

    print(f"成功打开摄像头 /dev/video{device_id}，按 q 键退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取视频帧")
            break

        cv2.imshow("Camera Test", frame)

        # 按下 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera(device_id=0)

