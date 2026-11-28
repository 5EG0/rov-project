"""
ROV Project - GUI Controller (Anggota 2)
File: gui/gui_controller.py

Mockup GUI dan event handler mandiri menggunakan Tkinter.
Mode simulasi aktif secara default. Tidak ada perintah yang dikirim ke
hardware kecuali adapter backend nyata diberikan secara eksplisit.

Integrasi yang disiapkan:
- send_movement(direction, speed): callback ke modul movement.py
- send_stop(): callback berhenti
- send_mode(mode): callback perubahan mode
- update_video(frame): menerima frame dari computer_vision/vision.py
- update_detection(data): menerima hasil deteksi CV

Arah gerakan yang digunakan sebagai kontrak awal:
forward, backward, left, right, up, down, yaw_left, yaw_right, stop
Silakan sepakati nama perintah ini dengan Anggota 3, 4, dan 5.
"""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Any, Callable, Optional


MovementCallback = Callable[[str, int], None]
StopCallback = Callable[[], None]
ModeCallback = Callable[[str], None]


class CameraWorker(threading.Thread):
    """Thread yang membaca kamera terus-menerus lalu mengirim frame ke GUI."""

    def __init__(self, index: int, on_frame: Callable[[Any], None],
                 on_error: Callable[[str], None]) -> None:
        super().__init__(daemon=True)
        self.index = index
        self.on_frame = on_frame
        self.on_error = on_error
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        try:
            import cv2
        except ImportError:
            self.on_error("opencv-python belum terinstall")
            return

        cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():
            self.on_error(f"Kamera index {self.index} tidak bisa dibuka")
            return

        while not self._stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                self.on_error("Gagal membaca frame dari kamera")
                break
            self.on_frame(frame)
            time.sleep(0.03)  # sekitar 30 FPS

        cap.release()


class GUIController:
    """Mengelola event GUI dan meneruskan perintah ke backend."""

    def __init__(
        self,
        root: tk.Tk,
        send_movement: Optional[MovementCallback] = None,
        send_stop: Optional[StopCallback] = None,
        send_mode: Optional[ModeCallback] = None,
        simulation: bool = True,
    ) -> None:
        self.root = root
        self.simulation = simulation

        # Callback None berarti simulasi/log saja, bukan kontrol hardware.
        self.movement_callback = send_movement
        self.stop_callback = send_stop
        self.mode_callback = send_mode

        self.connected = False
        self.control_mode = "MANUAL"
        self.speed = tk.IntVar(value=50)
        self.status_text = tk.StringVar(value="DISCONNECTED")
        self.mode_text = tk.StringVar(value=self.control_mode)
        self.detection_text = tk.StringVar(value="Belum ada data deteksi.")
        self.last_command_text = tk.StringVar(value="Belum ada perintah.")
        self.video_text = tk.StringVar(value="VIDEO FEED\nMenunggu kamera...")
        self.event_queue: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.active_direction: Optional[str] = None

        # Kamera (G9)
        self.camera_index = tk.IntVar(value=0)
        self.camera_worker: Optional[CameraWorker] = None
        self._latest_frame: Any = None

        self._build_gui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self._process_event_queue)

    # -------------------- GUI --------------------

    def _build_gui(self) -> None:
        self.root.title("ROV Control Station - Mockup")
        self.root.geometry("1050x700")
        self.root.minsize(850, 600)

        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)

        header = ttk.Frame(main)
        header.pack(fill="x", pady=(0, 10))
        ttk.Label(
            header, text="ROV CONTROL STATION",
            font=("Arial", 18, "bold")
        ).pack(side="left")
        ttk.Label(
            header, text="GUI Controller | Anggota 2",
            font=("Arial", 10)
        ).pack(side="right")

        status_frame = ttk.LabelFrame(main, text="Status Sistem", padding=8)
        status_frame.pack(fill="x", pady=(0, 10))

        ttk.Label(status_frame, text="Koneksi:").pack(side="left")
        ttk.Label(
            status_frame, textvariable=self.status_text,
            font=("Arial", 10, "bold")
        ).pack(side="left", padx=(5, 18))

        ttk.Label(status_frame, text="Mode:").pack(side="left")
        ttk.Label(
            status_frame, textvariable=self.mode_text,
            font=("Arial", 10, "bold")
        ).pack(side="left", padx=5)

        ttk.Button(
            status_frame, text="Connect", command=self.connect_rov
        ).pack(side="right", padx=4)
        ttk.Button(
            status_frame, text="Disconnect", command=self.disconnect_rov
        ).pack(side="right", padx=4)

        content = ttk.Frame(main)
        content.pack(fill="both", expand=True)
        content.columnconfigure(0, weight=3)
        content.columnconfigure(1, weight=2)
        content.rowconfigure(0, weight=1)

        # Panel video
        left = ttk.Frame(content)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left.rowconfigure(0, weight=3)
        left.rowconfigure(1, weight=2)
        left.columnconfigure(0, weight=1)

        video_panel = ttk.LabelFrame(left, text="Video ROV", padding=8)
        video_panel.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        video_panel.rowconfigure(0, weight=1)
        video_panel.columnconfigure(0, weight=1)

        self.video_label = tk.Label(
            video_panel, textvariable=self.video_text,
            bg="black", fg="white", font=("Arial", 14), justify="center"
        )
        self.video_label.grid(row=0, column=0, sticky="nsew")

        # Panel hasil deteksi
        detection_panel = ttk.LabelFrame(left, text="Hasil Deteksi CV", padding=8)
        detection_panel.grid(row=1, column=0, sticky="nsew")
        ttk.Label(
            detection_panel, textvariable=self.detection_text,
            justify="left", wraplength=500
        ).pack(anchor="nw")

        # Panel kontrol (kanan)
        right = ttk.Frame(content)
        right.grid(row=0, column=1, sticky="nsew")

        move_panel = ttk.LabelFrame(right, text="Kontrol Gerak", padding=8)
        move_panel.pack(fill="x", pady=(0, 8))
        for c in range(3):
            move_panel.columnconfigure(c, weight=1)

        buttons = [
            ("Yaw Kiri", "yaw_left", 0, 0),
            ("Maju", "forward", 0, 1),
            ("Yaw Kanan", "yaw_right", 0, 2),
            ("Kiri", "left", 1, 0),
            ("STOP", "stop", 1, 1),
            ("Kanan", "right", 1, 2),
            ("Naik", "up", 2, 0),
            ("Mundur", "backward", 2, 1),
            ("Turun", "down", 2, 2),
        ]
        for text, direction, r, c in buttons:
            ttk.Button(
                move_panel, text=text,
                command=lambda d=direction: self.on_move(d)
            ).grid(row=r, column=c, sticky="nsew", padx=3, pady=3, ipady=8)

        # Kecepatan
        speed_panel = ttk.LabelFrame(right, text="Kecepatan (%)", padding=8)
        speed_panel.pack(fill="x", pady=(0, 8))
        tk.Scale(
            speed_panel, from_=0, to=100, orient="horizontal",
            variable=self.speed
        ).pack(fill="x")

        # Mode
        mode_panel = ttk.LabelFrame(right, text="Mode Kontrol", padding=8)
        mode_panel.pack(fill="x", pady=(0, 8))
        self.mode_box = ttk.Combobox(
            mode_panel, values=["MANUAL", "AUTO"], state="readonly"
        )
        self.mode_box.set(self.control_mode)
        self.mode_box.pack(fill="x")
        self.mode_box.bind("<<ComboboxSelected>>", self.on_mode_change)

        # Kamera (G9)
        cam_panel = ttk.LabelFrame(right, text="Kamera", padding=8)
        cam_panel.pack(fill="x", pady=(0, 8))
        ttk.Label(cam_panel, text="Index:").pack(side="left")
        ttk.Spinbox(
            cam_panel, from_=0, to=5, width=4, textvariable=self.camera_index
        ).pack(side="left", padx=(4, 10))
        ttk.Button(
            cam_panel, text="Start Kamera", command=self.start_camera
        ).pack(side="left", padx=3)
        ttk.Button(
            cam_panel, text="Stop Kamera", command=self.stop_camera
        ).pack(side="left", padx=3)

        # Perintah terakhir
        cmd_panel = ttk.LabelFrame(right, text="Perintah Terakhir", padding=8)
        cmd_panel.pack(fill="x")
        ttk.Label(cmd_panel, textvariable=self.last_command_text).pack(anchor="w")

        # Shortcut: spasi = STOP
        self.root.bind("<space>", lambda e: self.on_stop())

    # -------------------- Event handler --------------------

    def connect_rov(self) -> None:
        self.connected = True
        label = "CONNECTED (SIMULASI)" if self.simulation else "CONNECTED"
        self.status_text.set(label)

    def disconnect_rov(self) -> None:
        self.on_stop()
        self.connected = False
        self.status_text.set("DISCONNECTED")

    def on_move(self, direction: str) -> None:
        if direction == "stop":
            self.on_stop()
            return
        if not self.connected:
            messagebox.showwarning("Belum terhubung", "Klik Connect dulu.")
            return
        speed = self.speed.get()
        if self.movement_callback:
            self.movement_callback(direction, speed)
        self.last_command_text.set(f"{direction} (speed {speed}%)")
        print(f"[GUI] {direction} speed={speed}")

    def on_stop(self) -> None:
        if self.stop_callback:
            self.stop_callback()
        self.last_command_text.set("stop")
        print("[GUI] stop")

    def on_mode_change(self, event=None) -> None:
        self.control_mode = self.mode_box.get()
        self.mode_text.set(self.control_mode)
        if self.mode_callback:
            self.mode_callback(self.control_mode)
        print(f"[GUI] mode={self.control_mode}")

    def on_close(self) -> None:
        self.stop_camera()
        self.on_stop()
        self.root.destroy()

    # -------------------- Kamera (G9) --------------------

    def start_camera(self) -> None:
        if self.camera_worker and self.camera_worker.is_alive():
            return  # sudah berjalan
        self.video_text.set("Membuka kamera...")
        self.camera_worker = CameraWorker(
            self.camera_index.get(),
            on_frame=self.update_video,
            on_error=self._camera_error,
        )
        self.camera_worker.start()

    def stop_camera(self) -> None:
        if self.camera_worker:
            self.camera_worker.stop()
            self.camera_worker = None
        self._latest_frame = None
        self.video_label.configure(image="")
        self.video_label.image = None
        self.video_text.set("VIDEO FEED\nKamera dimatikan")

    def _camera_error(self, message: str) -> None:
        # Dipanggil dari thread kamera, jadi lewat queue
        self.event_queue.put(("camera_error", message))

    # -------------------- Dipanggil modul lain --------------------

    def update_video(self, frame: Any) -> None:
        """Dipanggil dari vision.py / thread kamera. Aman dari thread lain.
        Hanya frame terbaru yang disimpan supaya GUI tidak lag."""
        self._latest_frame = frame

    def update_detection(self, data: Any) -> None:
        """Dipanggil dari vision.py untuk hasil deteksi."""
        self.event_queue.put(("detection", data))

    def _process_event_queue(self) -> None:
        try:
            while True:
                kind, payload = self.event_queue.get_nowait()
                if kind == "detection":
                    self.detection_text.set(str(payload))
                elif kind == "camera_error":
                    self.stop_camera()
                    self.video_text.set(f"ERROR\n{payload}")
        except queue.Empty:
            pass

        frame = self._latest_frame
        if frame is not None:
            self._latest_frame = None
            self._show_frame(frame)

        self.root.after(30, self._process_event_queue)

    def _show_frame(self, frame: Any) -> None:
        # Frame OpenCV (BGR) diubah jadi gambar Tkinter (butuh opencv-python & Pillow)
        try:
            import cv2
            from PIL import Image, ImageTk
        except ImportError:
            self.video_text.set("Install opencv-python & Pillow\nuntuk tampilkan video")
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)

        # Sesuaikan ukuran dengan panel, tetap menjaga rasio gambar
        w = max(self.video_label.winfo_width(), 320)
        h = max(self.video_label.winfo_height(), 240)
        img.thumbnail((w, h))

        photo = ImageTk.PhotoImage(img)
        self.video_label.configure(image=photo, text="")
        self.video_label.image = photo  # simpan supaya tidak hilang


if __name__ == "__main__":
    root = tk.Tk()
    app = GUIController(root)
    root.mainloop()
