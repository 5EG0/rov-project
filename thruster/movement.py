"""
Modul Control Logic & Integrasi Thruster ROV (4 Thruster: 2 Horizontal, 2 Vertikal)
PIC: Diko
File: ROV_Project/thruster/movement.py
"""

from dataclasses import dataclass
from typing import Dict, Optional


# Konstanta PWM ESC standar (mikrodetik)
PWM_MIN = 1100
PWM_NEUTRAL = 1500
PWM_MAX = 1900


@dataclass
class ThrusterOutput:
    """Representasi output untuk masing-masing dari 4 thruster."""
    h_left: float = 0.0   # Horizontal Kiri  (-1.0 s.d. 1.0)
    h_right: float = 0.0  # Horizontal Kanan (-1.0 s.d. 1.0)
    v_left: float = 0.0   # Vertikal Kiri    (-1.0 s.d. 1.0)
    v_right: float = 0.0  # Vertikal Kanan   (-1.0 s.d. 1.0)

    def to_normalized_dict(self) -> Dict[str, float]:
        return {
            "H_LEFT": round(self.h_left, 3),
            "H_RIGHT": round(self.h_right, 3),
            "V_LEFT": round(self.v_left, 3),
            "V_RIGHT": round(self.v_right, 3),
        }

    def to_pwm_dict(self) -> Dict[str, int]:
        return {
            "H_LEFT": thrust_to_pwm(self.h_left),
            "H_RIGHT": thrust_to_pwm(self.h_right),
            "V_LEFT": thrust_to_pwm(self.v_left),
            "V_RIGHT": thrust_to_pwm(self.v_right),
        }


def clamp(value: float, min_val: float = -1.0, max_val: float = 1.0) -> float:
    """Membatasi nilai agar tetap berada di rentang aman."""
    return max(min_val, min(max_val, value))


def thrust_to_pwm(thrust: float) -> int:
    """Konversi nilai thrust (-1.0 s.d. 1.0) menjadi sinyal PWM (1100 s.d. 1900 us)."""
    thrust = clamp(thrust, -1.0, 1.0)
    if thrust >= 0:
        return int(round(PWM_NEUTRAL + thrust * (PWM_MAX - PWM_NEUTRAL)))
    return int(round(PWM_NEUTRAL + thrust * (PWM_NEUTRAL - PWM_MIN)))


class ROVMovementController:
    """
    Pengendali logika gerak ROV.
    Mendukung mode simulasi (tanpa hardware) dan mode integrasi ke thruster/driver.py.
    """

    def __init__(self, simulation_mode: bool = True, driver=None):
        self.simulation_mode = simulation_mode
        self.driver = driver
        self.current_command = "STOP"
        self.current_output = ThrusterOutput()

    def compute_mixing(self, surge: float = 0.0, yaw: float = 0.0, heave: float = 0.0) -> ThrusterOutput:
        """
        Menghitung distribusi daya ke 4 thruster dari vektor gerak:
        - surge: maju (+) / mundur (-)
        - yaw  : belok kanan (+) / belok kiri (-)
        - heave: naik (+) / turun (-)
        """
        h_left = clamp(surge + yaw)
        h_right = clamp(surge - yaw)
        v_left = clamp(heave)
        v_right = clamp(heave)
        return ThrusterOutput(h_left=h_left, h_right=h_right, v_left=v_left, v_right=v_right)

    def execute_command(self, command: str, speed: float = 0.5) -> Dict[str, int]:
        """
        Menerima perintah diskrit dari GUI (misal: 'MAJU', 'KIRI', 'NAIK', 'STOP')
        dan menerjemahkannya ke sinyal thruster.
        """
        cmd = command.strip().upper()
        spd = clamp(abs(speed), 0.0, 1.0)

        command_map = {
            "MAJU":    (spd, 0.0, 0.0),
            "MUNDUR":  (-spd, 0.0, 0.0),
            "KANAN":   (0.0, spd, 0.0),
            "KIRI":    (0.0, -spd, 0.0),
            "NAIK":    (0.0, 0.0, spd),
            "TURUN":   (0.0, 0.0, -spd),
            "STOP":    (0.0, 0.0, 0.0),
        }

        if cmd not in command_map:
            raise ValueError(f"Perintah tidak dikenali: '{command}'. Gunakan: {list(command_map.keys())}")

        surge, yaw, heave = command_map[cmd]
        self.current_command = cmd
        self.current_output = self.compute_mixing(surge=surge, yaw=yaw, heave=heave)
        return self._dispatch()

    def execute_vector(self, surge: float = 0.0, yaw: float = 0.0, heave: float = 0.0) -> Dict[str, int]:
        """
        Menerima input kontinu/kombinasi (misal dari joystick atau auto-tracking CV).
        """
        self.current_command = f"VECTOR(surge={surge:.2f}, yaw={yaw:.2f}, heave={heave:.2f})"
        self.current_output = self.compute_mixing(surge=surge, yaw=yaw, heave=heave)
        return self._dispatch()

    def emergency_stop(self) -> Dict[str, int]:
        """Menghentikan seluruh thruster seketika."""
        return self.execute_command("STOP", speed=0.0)

    def get_status(self) -> dict:
        """Mengembalikan status terkini untuk ditampilkan di panel indikator GUI."""
        return {
            "mode": "SIMULASI" if self.simulation_mode else "HARDWARE",
            "command": self.current_command,
            "normalized": self.current_output.to_normalized_dict(),
            "pwm": self.current_output.to_pwm_dict(),
        }

    def _dispatch(self) -> Dict[str, int]:
        """Mengirim output ke simulator terminal atau driver hardware Seto."""
        pwm_values = self.current_output.to_pwm_dict()
        norm_values = self.current_output.to_normalized_dict()

        if self.simulation_mode or self.driver is None:
            print(f"[SIMULASI] Perintah: {self.current_command}")
            print(f"           Daya (%) : { {k: f'{v*100:+.0f}%' for k, v in norm_values.items()} }")
            print(f"           PWM (us) : {pwm_values}")
            print("-" * 65)
        else:
            # Siap dipanggil saat integrasi dengan thruster/driver.py milik Seto
            self.driver.set_thrusters(pwm_values)

        return pwm_values


# =====================================================================
# MODE SIMULASI MANDIRI (Jalankan langsung: python thruster/movement.py)
# =====================================================================
def run_auto_simulation():
    print("=== MULAI UJI SIMULASI OTOMATIS 4 THRUSTER ROV ===\n")
    rov = ROVMovementController(simulation_mode=True)

    skenario = [
        ("MAJU", 0.6),
        ("MUNDUR", 0.4),
        ("KANAN", 0.5),
        ("KIRI", 0.5),
        ("NAIK", 0.7),
        ("TURUN", 0.3),
        ("STOP", 0.0),
    ]

    for cmd, spd in skenario:
        rov.execute_command(cmd, speed=spd)

    print("Uji gerak kombinasi (Maju 60% sambil Belok Kanan 30% dan Naik 40%):")
    rov.execute_vector(surge=0.6, yaw=0.3, heave=0.4)
    rov.emergency_stop()


def run_interactive_simulation():
    print("=== SIMULASI INTERAKTIF KONTROL ROV (PENGGANTI GUI) ===")
    print("Kontrol: [w] Maju | [s] Mundur | [a] Kiri | [d] Kanan | [q] Naik | [e] Turun | [x] Stop | [exit] Keluar\n")
    rov = ROVMovementController(simulation_mode=True)

    keymap = {
        "w": "MAJU",
        "s": "MUNDUR",
        "a": "KIRI",
        "d": "KANAN",
        "q": "NAIK",
        "e": "TURUN",
        "x": "STOP",
    }

    while True:
        tombol = input("Input tombol GUI > ").strip().lower()
        if tombol == "exit":
            rov.emergency_stop()
            print("Simulasi selesai.")
            break
        if tombol in keymap:
            rov.execute_command(keymap[tombol], speed=0.5)
        else:
            print("Tombol tidak valid. Gunakan w/a/s/d/q/e/x atau 'exit'.")


if __name__ == "__main__":
    print("Pilih mode uji simulasi:")
    print("1. Uji Otomatis Semua Skenario Gerak")
    print("2. Uji Interaktif via Keyboard (Simulasi Input GUI)")
    pilihan = input("Masukkan pilihan (1/2): ").strip()

    if pilihan == "2":
        run_interactive_simulation()
    else:
        run_auto_simulation()
