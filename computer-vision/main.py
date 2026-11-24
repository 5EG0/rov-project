import cv2
from detector import UnderwaterDetector


def main():
    # Inisialisasi detector (otomatis unduh yolo26n.pt saat pertama kali jalan)
    detector = UnderwaterDetector(model_path="yolo26n.pt", conf_threshold=0.5)

    cap = cv2.VideoCapture(0)  # 0 = webcam laptop, atau ganti "video_uji.mp4"

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        annotated_frame, target_info = detector.process_frame(frame)

        if target_info["detected"]:
            print(
                f"Target: {target_info['label']} | "
                f"err_x: {target_info['error_x']} | "
                f"err_y: {target_info['error_y']}"
            )

        cv2.imshow("ROV Computer Vision Test", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()