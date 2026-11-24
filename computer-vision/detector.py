import cv2
from ultralytics import YOLO


class UnderwaterDetector:
    def __init__(self, model_path="yolo26n.pt", conf_threshold=0.5):
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold

    def process_frame(self, frame):
        """
        Memproses 1 frame gambar dari kamera ROV.
        Return:
        - annotated_frame: frame dengan bounding box (untuk GUI)
        - target_info: dictionary koordinat & error objek (untuk Thruster)
        """
        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2

        results = self.model.predict(
            frame, conf=self.conf_threshold, verbose=False
        )
        annotated_frame = results[0].plot()

        # Gambar titik tengah kamera (crosshair) sebagai referensi
        cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 255, 0), -1)

        target_info = {
            "detected": False,
            "label": None,
            "confidence": 0.0,
            "error_x": 0,  # Negatif = objek di kiri, Positif = di kanan
            "error_y": 0,  # Negatif = objek di atas, Positif = di bawah
            "area": 0,  # Estimasi seberapa dekat objek dengan ROV
        }

        boxes = results[0].boxes
        if len(boxes) > 0:
            # Ambil objek dengan confidence tertinggi (index 0)
            best_box = boxes[0]
            x1, y1, x2, y2 = best_box.xyxy[0].tolist()
            cls_id = int(best_box.cls[0])
            conf = float(best_box.conf[0])

            obj_x = int((x1 + x2) / 2)
            obj_y = int((y1 + y2) / 2)
            area = int((x2 - x1) * (y2 - y1))

            # Gambar garis dari tengah layar ke tengah objek
            cv2.line(
                annotated_frame,
                (center_x, center_y),
                (obj_x, obj_y),
                (0, 255, 255),
                2,
            )

            target_info = {
                "detected": True,
                "label": self.model.names[cls_id],
                "confidence": round(conf, 2),
                "error_x": obj_x - center_x,
                "error_y": obj_y - center_y,
                "area": area,
            }

        return annotated_frame, target_info