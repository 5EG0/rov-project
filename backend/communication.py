"""
Kontrak Komunikasi Antar Modul ROV (GUI, Computer Vision, Thruster)
File: ROV_Project/backend/communication.py

Digunakan bersama oleh:
- Christian (gui/dashboard.py)
- Hazza     (gui/gui_controller.py)
- Niken     (computer_vision/vision.py)
- Seto      (thruster/driver.py)
- Diko      (thruster/movement.py)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple


# =====================================================================
# 1. KONSTANTA STANDAR SISTEM (Disepakati Seluruh Tim)
# =====================================================================

THRUSTER_IDS: Tuple[str, ...] = ("H_LEFT", "H_RIGHT", "V_LEFT", "V_RIGHT")

PWM_MIN: int = 1100       # Full mundur / turun
PWM_NEUTRAL: int = 1500   # Berhenti (0%)
PWM_MAX: int = 1900       # Full maju / naik


class MoveDirection(str, Enum):
    """Daftar perintah gerak diskrit standar dari tombol GUI."""
    MAJU = "MAJU"
    MUNDUR = "MUNDUR"
    KANAN = "KANAN"
    KIRI = "KIRI"
    NAIK = "NAIK"
    TURUN = "TURUN"
    STOP = "STOP"
    VECTOR = "VECTOR"


class SystemMode(str, Enum):
    """Mode operasi sistem ROV."""
    SIMULATION = "SIMULASI"
    HARDWARE = "HARDWARE"


# =====================================================================
# 2. KONTRAK DATA: GUI (Hazza) -> MOVEMENT LOGIC (Diko)
# =====================================================================

@dataclass
class MotionCommand:
    """
    Format perintah gerak yang dikirim GUI (atau mode auto-CV) ke movement.py.
    - Untuk tombol GUI biasa: isi `direction` (misal MoveDirection.MAJU) dan `speed` (0.0 - 1.0).
    - Untuk joystick / gerak kombinasi: set `direction = MoveDirection.VECTOR` dan isi `surge`, `yaw`, `heave`.
    """
    direction: MoveDirection = MoveDirection.STOP
    speed: float = 0.5          # Rentang 0.0 s.d. 1.0 (untuk perintah diskrit)
    surge: float = 0.0          # Maju (+) / Mundur (-), rentang -1.0 s.d. 1.0
    yaw: float = 0.0            # Belok Kanan (+) / Kiri (-), rentang -1.0 s.d. 1.0
    heave: float = 0.0          # Naik (+) / Turun (-), rentang -1.0 s.d. 1.0
    source: str = "GUI"         # Asal perintah: "GUI", "CV_AUTO", atau "KEYBOARD"
    timestamp: float = field(default_factory=time.time)

    def validate(self) -> None:
        if not (0.0 <= self.speed <= 1.0):
            raise ValueError(f"speed harus di antara 0.0 dan 1.0, diterima: {self.speed}")
        for axis_name, val in (("surge", self.surge), ("yaw", self.yaw), ("heave", self.heave)):
            if not (-1.0 <= val <= 1.0):
                raise ValueError(f"{axis_name} harus di antara -1.0 dan 1.0, diterima: {val}")


# =====================================================================
# 3. KONTRAK DATA: MOVEMENT LOGIC (Diko) -> THRUSTER DRIVER (Seto)
# =====================================================================

@dataclass
class ThrusterPWMCommand:
    """
    Paket sinyal PWM akhir yang wajib diterima driver.py milik Seto.
    Semua nilai dijamin dalam rentang aman 1100 - 1900 us.
    """
    H_LEFT: int = PWM_NEUTRAL
    H_RIGHT: int = PWM_NEUTRAL
    V_LEFT: int = PWM_NEUTRAL
    V_RIGHT: int = PWM_NEUTRAL
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, int]:
        return {
            "H_LEFT": self.H_LEFT,
            "H_RIGHT": self.H_RIGHT,
            "V_LEFT": self.V_LEFT,
            "V_RIGHT": self.V_RIGHT,
        }

    @classmethod
    def from_dict(cls, pwm_map: Dict[str, int]) -> "ThrusterPWMCommand":
        for key in THRUSTER_IDS:
            if key not in pwm_map:
                raise KeyError(f"Key thruster '{key}' hilang dari dictionary PWM.")
            val = int(pwm_map[key])
            if not (PWM_MIN <= val <= PWM_MAX):
                raise ValueError(f"PWM {key}={val} di luar batas aman ({PWM_MIN}..{PWM_MAX}).")
        return cls(
            H_LEFT=int(pwm_map["H_LEFT"]),
            H_RIGHT=int(pwm_map["H_RIGHT"]),
            V_LEFT=int(pwm_map["V_LEFT"]),
            V_RIGHT=int(pwm_map["V_RIGHT"]),
        )


# =====================================================================
# 4. KONTRAK DATA: TELEMETRI THRUSTER -> GUI DASHBOARD (Hazza & Christian)
# =====================================================================

@dataclass
class ThrusterTelemetry:
    """Data status thruster untuk ditampilkan pada panel indikator GUI."""
    mode: SystemMode = SystemMode.SIMULATION
    armed: bool = False
    hardware_connected: bool = False
    active_command: str = "STOP"
    normalized_power: Dict[str, float] = field(
        default_factory=lambda: {k: 0.0 for k in THRUSTER_IDS}
    )
    pwm_values: Dict[str, int] = field(
        default_factory=lambda: {k: PWM_NEUTRAL for k in THRUSTER_IDS}
    )
    last_error: Optional[str] = None
    timestamp: float = field(default_factory=time.time)


# =====================================================================
# 5. KONTRAK DATA: COMPUTER VISION (Niken) -> GUI (Hazza)
# =====================================================================

@dataclass
class DetectedObject:
    """Satu objek hasil deteksi OpenCV."""
    label: str
    confidence: float                     # 0.0 s.d. 1.0
    bbox: Tuple[int, int, int, int]       # (x, y, width, height) dalam piksel
    center_offset: Tuple[float, float] = (0.0, 0.0)  # Offset relatif pusat frame (-1.0..1.0)


@dataclass
class VisionResult:
    """Paket hasil pemrosesan tiap frame dari computer_vision/vision.py."""
    frame_id: int = 0
    fps: float = 0.0
    camera_connected: bool = False
    detections: List[DetectedObject] = field(default_factory=list)
    annotated_frame: Any = None           # Frame numpy/BGR siap tampil di GUI
    timestamp: float = field(default_factory=time.time)


# =====================================================================
# 6. KONTRAK INTERFACE (Class Induk yang Wajib Diikuti Seto & Diko)
# =====================================================================

class BaseThrusterDriver(ABC):
    """
    Kontrak method untuk thruster/driver.py (Seto).
    movement.py hanya akan memanggil method-method di bawah ini.
    """

    @abstractmethod
    def connect(self) -> bool:
        """Membuka koneksi ke flight controller / mikrokontroler ESC."""
        pass

    @abstractmethod
    def arm(self) -> bool:
        """Mengaktifkan (arming) ESC agar motor siap menerima PWM."""
        pass

    @abstractmethod
    def disarm(self) -> None:
        """Menonaktifkan motor dan mengembalikan PWM ke netral (1500 us)."""
        pass

    @abstractmethod
    def set_thrusters(self, pwm_values: Dict[str, int]) -> None:
        """
        Mengirim sinyal PWM ke 4 motor.
        Format wajib: {'H_LEFT': int, 'H_RIGHT': int, 'V_LEFT': int, 'V_RIGHT': int}
        """
        pass

    @abstractmethod
    def stop_all(self) -> None:
        """Menghentikan seluruh motor ke PWM_NEUTRAL (1500 us)."""
        pass


class BaseMovementController(ABC):
    """
    Kontrak method untuk thruster/movement.py (Diko).
    gui_controller.py milik Hazza akan memanggil method-method di bawah ini.
    """

    @abstractmethod
    def handle_motion_command(self, cmd: MotionCommand) -> ThrusterTelemetry:
        """Memproses objek MotionCommand dari GUI/CV dan mengembalikan telemetri terbaru."""
        pass

    @abstractmethod
    def emergency_stop(self) -> ThrusterTelemetry:
        """Menghentikan seluruh thruster secara darurat."""
        pass

    @abstractmethod
    def get_telemetry(self) -> ThrusterTelemetry:
        """Mengambil snapshot status thruster terkini untuk GUI."""
        pass


# =====================================================================
# 7. HUB KOMUNIKASI TERPUSAT (Penghubung Antar Thread/Modul)
# =====================================================================

class ROVCommunicationHub:
    """
    Pusat pertukaran data thread-safe antar modul GUI, CV, dan Thruster.
    Dapat diinisialisasi di main.py lalu dibagikan ke seluruh modul.
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._movement_controller: Optional[BaseMovementController] = None
        self._latest_telemetry = ThrusterTelemetry()
        self._latest_vision = VisionResult()
        self._telemetry_listeners: List[Callable[[ThrusterTelemetry], None]] = []
        self._vision_listeners: List[Callable[[VisionResult], None]] = []

    # --- Registrasi Modul ---
    def register_movement_controller(self, controller: BaseMovementController) -> None:
        with self._lock:
            self._movement_controller = controller
            self._latest_telemetry = controller.get_telemetry()

    def subscribe_telemetry(self, callback: Callable[[ThrusterTelemetry], None]) -> None:
        """GUI mendaftarkan fungsi untuk menerima update status thruster otomatis."""
        with self._lock:
            self._telemetry_listeners.append(callback)

    def subscribe_vision(self, callback: Callable[[VisionResult], None]) -> None:
        """GUI mendaftarkan fungsi untuk menerima frame & hasil deteksi CV."""
        with self._lock:
            self._vision_listeners.append(callback)

    # --- Pengiriman Perintah & Data ---
    def send_motion_command(self, cmd: MotionCommand) -> ThrusterTelemetry:
        """Dipanggil oleh Hazza (gui_controller.py) saat tombol ditekan."""
        cmd.validate()
        with self._lock:
            if self._movement_controller is None:
                raise RuntimeError("MovementController belum didaftarkan ke ROVCommunicationHub.")
            telemetry = self._movement_controller.handle_motion_command(cmd)
            self._latest_telemetry = telemetry
            listeners = list(self._telemetry_listeners)

        for cb in listeners:
            cb(telemetry)
        return telemetry

    def trigger_emergency_stop(self) -> ThrusterTelemetry:
        """Dipanggil oleh Hazza saat tombol EMERGENCY STOP ditekan."""
        with self._lock:
            if self._movement_controller is None:
                self._latest_telemetry = ThrusterTelemetry(active_command="EMERGENCY_STOP")
                return self._latest_telemetry
            telemetry = self._movement_controller.emergency_stop()
            self._latest_telemetry = telemetry
            listeners = list(self._telemetry_listeners)

        for cb in listeners:
            cb(telemetry)
        return telemetry

    def publish_vision_result(self, result: VisionResult) -> None:
        """Dipanggil oleh Niken (vision.py) setiap selesai memproses frame kamera."""
        with self._lock:
            self._latest_vision = result
            listeners = list(self._vision_listeners)

        for cb in listeners:
            cb(result)

    # --- Pengambilan State Terkini (Polling) ---
    def get_latest_telemetry(self) -> ThrusterTelemetry:
        with self._lock:
            return self._latest_telemetry

    def get_latest_vision(self) -> VisionResult:
        with self._lock:
            return self._latest_vision


# =====================================================================
# UJI INTEGRASI KONTRAK MANDIRI (python backend/communication.py)
# =====================================================================
if __name__ == "__main__":
    class DummyMovement(BaseMovementController):
        def __init__(self) -> None:
            self.telemetry = ThrusterTelemetry(mode=SystemMode.SIMULATION, armed=True)

        def handle_motion_command(self, cmd: MotionCommand) -> ThrusterTelemetry:
            self.telemetry.active_command = cmd.direction.value
            if cmd.direction == MoveDirection.MAJU:
                pwm = int(PWM_NEUTRAL + cmd.speed * 400)
                self.telemetry.pwm_values = {
                    "H_LEFT": pwm, "H_RIGHT": pwm,
                    "V_LEFT": PWM_NEUTRAL, "V_RIGHT": PWM_NEUTRAL,
                }
            elif cmd.direction == MoveDirection.STOP:
                self.telemetry.pwm_values = {k: PWM_NEUTRAL for k in THRUSTER_IDS}
            return self.telemetry

        def emergency_stop(self) -> ThrusterTelemetry:
            self.telemetry.active_command = "EMERGENCY_STOP"
            self.telemetry.pwm_values = {k: PWM_NEUTRAL for k in THRUSTER_IDS}
            return self.telemetry

        def get_telemetry(self) -> ThrusterTelemetry:
            return self.telemetry

    hub = ROVCommunicationHub()
    hub.register_movement_controller(DummyMovement())

    # Simulasi Hazza di GUI berlangganan update telemetri
    hub.subscribe_telemetry(
        lambda t: print(f"[GUI Update] Cmd={t.active_command} | PWM={t.pwm_values}")
    )

    # Simulasi Hazza menekan tombol MAJU 50% lalu STOP
    hub.send_motion_command(MotionCommand(direction=MoveDirection.MAJU, speed=0.5))
    hub.trigger_emergency_stop()
