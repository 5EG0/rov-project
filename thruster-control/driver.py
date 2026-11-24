"""
driver.py - Driver thruster ROV (4 thruster, ESC bidirectional 40A)

Jalur kontrol:
    Mini PC (driver.py) --MAVLink/serial--> Pixhawk 6C (PX4) --PWM--> 4 ESC --> 4 motor

Cara kerja:
    Driver mengirim MAV_CMD_DO_SET_ACTUATOR (param1..param4 = Offboard Actuator Set 1..4)
    dengan nilai -1.0 .. +1.0 (0 = netral / motor berhenti).
    Konversi nilai -> PWM (mis. 1100/1500/1900 us) dilakukan OLEH PIXHAWK
    sesuai pengaturan PWM Min / Max / Disarmed di QGroundControl (Actuators).

Setup wajib di QGroundControl (Vehicle Setup > Actuators):
    - Output yang dicolok ESC  -> fungsi "Offboard Actuator Set 1..4"
      (sesuai ACTUATOR_MAP di bawah)
    - PWM Min = 1100, PWM Max = 1900, PWM Disarmed = 1500  (sesuaikan dengan ESC)
    - Kalibrasi ESC mengikuti prosedur ESC (netral 1500 us untuk ESC dua arah)

Antarmuka untuk movement.py:
    with ThrusterDriver() as drv:
        drv.arm()
        drv.set_thruster(1, 0.5)          # thruster 1 maju 50%
        drv.set_all(0.3, 0.3, 0.0, 0.0)   # atau semua sekaligus
        drv.stop_all()
        drv.disarm()

PENTING: perintah harus dikirim ulang minimal tiap < COMMAND_TIMEOUT_S detik.
Kalau tidak ada perintah baru, semua thruster otomatis berhenti (failsafe).
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import threading
import time
from typing import Dict, List, Optional, Sequence

# =============================================================================
# KONFIGURASI  (ubah di sini; semua nilai bisa disesuaikan nanti)
# =============================================================================

# --- Mode ---------------------------------------------------------------------
SIMULATE = False                 # True = tanpa Pixhawk (hanya log, untuk uji logic)

# --- Koneksi Mini PC <-> Pixhawk ------------------------------------------------
CONNECTION_TYPE = "serial"       # "serial" | "udp" | "tcp"
CONNECTION_PORT = os.environ.get("THRUSTER_PORT", "COM3")   # serial: COM3 (Windows) / /dev/ttyACM0 (Linux)
CONNECTION_BAUD = 115200         # untuk USB biasanya tidak berpengaruh; untuk telemetry serial sesuaikan
CONNECTION_HOST = "127.0.0.1"    # dipakai jika CONNECTION_TYPE = "udp" / "tcp"
CONNECTION_NET_PORT = 14550      # dipakai jika CONNECTION_TYPE = "udp" / "tcp"
HEARTBEAT_WAIT_S = 10.0          # batas waktu menunggu heartbeat Pixhawk saat connect

# --- Thruster -------------------------------------------------------------------
NUM_THRUSTERS = 4
# Thruster (1..4) -> nomor "Offboard Actuator Set" (1..6) di QGroundControl
ACTUATOR_MAP: Dict[int, int] = {1: 1, 2: 2, 3: 3, 4: 4}
# True = arah putar thruster dibalik (jika arah putar motor terbalik dari yang diharapkan)
REVERSED: Dict[int, bool] = {1: False, 2: False, 3: False, 4: False}

# --- PWM ESC (hanya referensi untuk setting QGC & log; PWM sebenarnya dibuat Pixhawk) ---
PWM_MIN_US = 1100                # mundur penuh
PWM_NEUTRAL_US = 1500            # berhenti
PWM_MAX_US = 1900                # maju penuh

# --- Batas keamanan -------------------------------------------------------------
MAX_THRUST = 0.5                 # 0..1. Nilai perintah 1.0 = MAX_THRUST dari kemampuan penuh (aman untuk uji awal)
DEADBAND = 0.03                  # |nilai| di bawah ini dianggap 0 (hindari motor berdenyut)
RAMP_PER_SEC = 1.5               # perubahan maksimum nilai per detik (halus; lindungi ESC & PSU)
COMMAND_TIMEOUT_S = 1.0          # tanpa perintah baru selama ini -> semua thruster berhenti
SEND_RATE_HZ = 20                # frekuensi pengiriman perintah ke Pixhawk
HEARTBEAT_RATE_HZ = 1            # frekuensi heartbeat GCS dari driver
ACK_TIMEOUT_S = 3.0              # batas waktu menunggu COMMAND_ACK saat arm/disarm

# =============================================================================
# Konstanta MAVLink
# =============================================================================
MAV_CMD_DO_SET_ACTUATOR = 187   # dipakai nomor langsung, karena beberapa versi pymavlink belum punya konstantanya
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_RESULT_ACCEPTED = 0
FORCE_ARM_MAGIC = 21196          # param2 untuk force arm/disarm (PX4)

log = logging.getLogger("thruster.driver")


class ThrusterError(Exception):
    """Error umum driver thruster."""


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def thrust_to_pwm_us(value: float) -> int:
    """Perkiraan PWM (us) untuk nilai -1..1 (hanya untuk log/diagnostik)."""
    value = _clamp(value, -1.0, 1.0)
    if value >= 0:
        return round(PWM_NEUTRAL_US + value * (PWM_MAX_US - PWM_NEUTRAL_US))
    return round(PWM_NEUTRAL_US + value * (PWM_NEUTRAL_US - PWM_MIN_US))


class ThrusterDriver:
    """Driver 4 thruster via Pixhawk (PX4) memakai MAV_CMD_DO_SET_ACTUATOR."""

    def __init__(
        self,
        simulate: Optional[bool] = None,
        port: Optional[str] = None,
        baud: Optional[int] = None,
    ) -> None:
        self.simulate = SIMULATE if simulate is None else simulate
        self.port = port or CONNECTION_PORT
        self.baud = baud or CONNECTION_BAUD

        self._conn = None
        self._target_system = 1
        self._target_component = 1

        self._lock = threading.Lock()      # melindungi state target/current
        self._io_lock = threading.Lock()   # melindungi akses koneksi MAVLink
        self._target: List[float] = [0.0] * NUM_THRUSTERS
        self._current: List[float] = [0.0] * NUM_THRUSTERS
        self._last_cmd_time = time.monotonic()
        self._timed_out = False
        self._armed = False

        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

        # nilai terakhir yang dikirim ke Pixhawk (berguna untuk uji/simulasi)
        self.last_sent: List[float] = [0.0] * NUM_THRUSTERS

        self._validate_config()

    # ------------------------------------------------------------------ config
    @staticmethod
    def _validate_config() -> None:
        for tid in range(1, NUM_THRUSTERS + 1):
            if tid not in ACTUATOR_MAP:
                raise ThrusterError(f"ACTUATOR_MAP tidak punya thruster {tid}")
            if not 1 <= ACTUATOR_MAP[tid] <= 6:
                raise ThrusterError("Nomor Offboard Actuator Set harus 1..6")
        if len(set(ACTUATOR_MAP[t] for t in range(1, NUM_THRUSTERS + 1))) != NUM_THRUSTERS:
            raise ThrusterError("ACTUATOR_MAP berisi nomor actuator yang kembar")
        if not 0.0 < MAX_THRUST <= 1.0:
            raise ThrusterError("MAX_THRUST harus di antara 0 (tidak termasuk) dan 1")

    # --------------------------------------------------------------- koneksi
    def connect(self) -> None:
        """Buka koneksi ke Pixhawk (atau mode simulasi) dan jalankan thread pengirim."""
        if self._thread is not None:
            return

        if self.simulate:
            log.info("[SIM] Mode simulasi aktif - tidak terhubung ke Pixhawk")
        else:
            try:
                from pymavlink import mavutil  # import malas: mode simulasi tidak butuh pymavlink
            except ImportError as exc:
                raise ThrusterError("pymavlink belum terpasang. Jalankan: pip install pymavlink pyserial") from exc

            if CONNECTION_TYPE == "serial":
                device = self.port
            elif CONNECTION_TYPE == "udp":
                device = f"udpin:{CONNECTION_HOST}:{CONNECTION_NET_PORT}"
            elif CONNECTION_TYPE == "tcp":
                device = f"tcp:{CONNECTION_HOST}:{CONNECTION_NET_PORT}"
            else:
                raise ThrusterError(f"CONNECTION_TYPE tidak dikenal: {CONNECTION_TYPE!r}")

            log.info("Menghubungkan ke Pixhawk: %s", device)
            try:
                self._conn = mavutil.mavlink_connection(device, baud=self.baud, source_system=255)
            except Exception as exc:
                raise ThrusterError(f"Gagal membuka koneksi {device}: {exc}") from exc

            hb = self._conn.wait_heartbeat(timeout=HEARTBEAT_WAIT_S)
            if hb is None:
                self._conn.close()
                self._conn = None
                raise ThrusterError(
                    f"Tidak ada heartbeat dari Pixhawk dalam {HEARTBEAT_WAIT_S}s "
                    f"(cek port/kabel/baudrate: {device})"
                )
            self._target_system = self._conn.target_system
            self._target_component = self._conn.target_component
            log.info("Terhubung. system=%s component=%s", self._target_system, self._target_component)

        self._stop_evt.clear()
        self._last_cmd_time = time.monotonic()
        self._thread = threading.Thread(target=self._loop, name="thruster-sender", daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        """Hentikan thruster, disarm (jika perlu), tutup koneksi."""
        if self._thread is None and self._conn is None:
            return
        try:
            self.stop_all()
            if self._armed:
                self.disarm()
        except Exception as exc:  # jangan sampai gagal membersihkan
            log.warning("Gagal disarm saat disconnect: %s", exc)

        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._conn is not None:
            with self._io_lock:
                try:
                    self._conn.close()
                finally:
                    self._conn = None
        log.info("Driver thruster ditutup")

    def __enter__(self) -> "ThrusterDriver":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # ------------------------------------------------------------ arm / disarm
    @property
    def armed(self) -> bool:
        return self._armed

    def arm(self, force: bool = False) -> bool:
        """Arm Pixhawk. force=True melewati preflight check (hati-hati)."""
        self._require_connected()
        ok = self._send_arm_command(True, force)
        self._armed = ok
        return ok

    def disarm(self, force: bool = False) -> bool:
        """Hentikan thruster lalu disarm Pixhawk."""
        self.stop_all()
        ok = self._send_arm_command(False, force)
        if ok:
            self._armed = False
        return ok

    def _send_arm_command(self, arm: bool, force: bool) -> bool:
        label = "ARM" if arm else "DISARM"
        if self.simulate:
            log.info("[SIM] %s", label)
            return True

        param2 = FORCE_ARM_MAGIC if force else 0
        with self._io_lock:
            self._conn.mav.command_long_send(
                self._target_system, self._target_component,
                MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1.0 if arm else 0.0, float(param2), 0, 0, 0, 0, 0,
            )
            deadline = time.monotonic() + ACK_TIMEOUT_S
            while time.monotonic() < deadline:
                msg = self._conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
                if msg is None or msg.command != MAV_CMD_COMPONENT_ARM_DISARM:
                    continue
                if msg.result == MAV_RESULT_ACCEPTED:
                    log.info("%s diterima Pixhawk", label)
                    return True
                log.error("%s ditolak Pixhawk (result=%s). Cek preflight / safety switch.", label, msg.result)
                return False
        log.error("%s: tidak ada COMMAND_ACK dalam %.1fs", label, ACK_TIMEOUT_S)
        return False

    # ------------------------------------------------------------ perintah thrust
    def set_thruster(self, thruster_id: int, value: float) -> None:
        """Set satu thruster. value -1.0 (mundur penuh) .. +1.0 (maju penuh), 0 = berhenti."""
        self._check_id(thruster_id)
        scaled = self._shape(value)
        with self._lock:
            self._target[thruster_id - 1] = scaled
            self._mark_command()

    def set_all(self, *values: float) -> None:
        """Set semua thruster sekaligus: set_all(v1, v2, v3, v4) atau set_all([v1, v2, v3, v4])."""
        if len(values) == 1 and isinstance(values[0], (list, tuple)):
            values = tuple(values[0])
        if len(values) != NUM_THRUSTERS:
            raise ValueError(f"set_all butuh {NUM_THRUSTERS} nilai, diterima {len(values)}")
        scaled = [self._shape(v) for v in values]
        with self._lock:
            self._target = scaled
            self._mark_command()

    def stop_all(self) -> None:
        """Hentikan semua thruster SEGERA (tanpa ramp)."""
        with self._lock:
            self._target = [0.0] * NUM_THRUSTERS
            self._current = [0.0] * NUM_THRUSTERS
            self._mark_command()
        try:
            self._send(self._current_snapshot())
        except Exception as exc:
            log.warning("Gagal mengirim stop: %s", exc)

    def get_state(self) -> dict:
        """Status ringkas untuk GUI / debug."""
        with self._lock:
            cur = list(self._current)
            tgt = list(self._target)
        return {
            "connected": self._thread is not None,
            "simulate": self.simulate,
            "armed": self._armed,
            "timed_out": self._timed_out,
            "target": tgt,
            "current": cur,
            "pwm_us": [thrust_to_pwm_us(v) for v in cur],
        }

    # ------------------------------------------------------------- internal
    def _mark_command(self) -> None:
        # dipanggil saat self._lock sudah dipegang
        self._last_cmd_time = time.monotonic()
        self._timed_out = False

    @staticmethod
    def _check_id(thruster_id: int) -> None:
        if not isinstance(thruster_id, int) or not 1 <= thruster_id <= NUM_THRUSTERS:
            raise ValueError(f"thruster_id harus 1..{NUM_THRUSTERS}, diterima {thruster_id!r}")

    @staticmethod
    def _shape(value: float) -> float:
        """Validasi, clamp, deadband, lalu skala dengan MAX_THRUST."""
        v = float(value)
        if math.isnan(v) or math.isinf(v):
            raise ValueError(f"Nilai thrust tidak valid: {value!r}")
        v = _clamp(v, -1.0, 1.0)
        if abs(v) < DEADBAND:
            return 0.0
        return v * MAX_THRUST

    def _current_snapshot(self) -> List[float]:
        with self._lock:
            return list(self._current)

    def _require_connected(self) -> None:
        if self._thread is None:
            raise ThrusterError("Driver belum connect(). Panggil connect() atau pakai 'with ThrusterDriver()'.")

    def _send(self, values: Sequence[float]) -> None:
        """Kirim nilai thruster (sudah dalam urutan thruster 1..N) ke Pixhawk."""
        params = [0.0] * 6
        for tid in range(1, NUM_THRUSTERS + 1):
            v = float(values[tid - 1])
            if REVERSED.get(tid, False):
                v = -v
            params[ACTUATOR_MAP[tid] - 1] = _clamp(v, -1.0, 1.0)

        self.last_sent = [float(v) for v in values]

        if self.simulate:
            log.debug("[SIM] actuator=%s pwm~%s", params, [thrust_to_pwm_us(v) for v in values])
            return
        if self._conn is None:
            return
        with self._io_lock:
            self._conn.mav.command_long_send(
                self._target_system, self._target_component,
                MAV_CMD_DO_SET_ACTUATOR, 0,
                params[0], params[1], params[2], params[3], params[4], params[5],
                0,  # param7 = index set (0 = actuator 1..6)
            )

    def _send_heartbeat(self) -> None:
        if self.simulate or self._conn is None:
            return
        from pymavlink import mavutil
        with self._io_lock:
            self._conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )

    def _loop(self) -> None:
        """Thread: failsafe watchdog + ramp + kirim perintah + heartbeat."""
        period = 1.0 / SEND_RATE_HZ
        hb_period = 1.0 / HEARTBEAT_RATE_HZ
        last = time.monotonic()
        last_hb = 0.0

        while not self._stop_evt.wait(period):
            now = time.monotonic()
            dt = now - last
            last = now

            with self._lock:
                # Failsafe: tidak ada perintah baru -> berhenti seketika
                if now - self._last_cmd_time > COMMAND_TIMEOUT_S:
                    if not self._timed_out and (any(self._target) or any(self._current)):
                        log.warning("Timeout perintah (> %.1fs) - semua thruster dihentikan", COMMAND_TIMEOUT_S)
                    self._timed_out = True
                    self._target = [0.0] * NUM_THRUSTERS
                    self._current = [0.0] * NUM_THRUSTERS
                else:
                    # Ramp: geser current menuju target dengan laju terbatas
                    step = RAMP_PER_SEC * dt
                    for i in range(NUM_THRUSTERS):
                        diff = self._target[i] - self._current[i]
                        self._current[i] += _clamp(diff, -step, step)
                values = list(self._current)

            try:
                self._send(values)
                if now - last_hb >= hb_period:
                    self._send_heartbeat()
                    last_hb = now
            except Exception as exc:
                log.error("Gagal kirim ke Pixhawk: %s", exc)


# =============================================================================
# Uji dari command line
# =============================================================================
def _hold(drv: ThrusterDriver, thruster_id: int, value: float, seconds: float) -> None:
    """Tahan nilai tertentu sambil mengirim ulang perintah (agar failsafe timeout tidak aktif)."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        drv.set_thruster(thruster_id, value)
        time.sleep(0.2)


def _bench_test(drv: ThrusterDriver, level: float, hold_s: float, force_arm: bool) -> None:
    print("\n=== UJI BANGKU THRUSTER ===")
    print("PASTIKAN PROPELLER SUDAH DILEPAS dan motor tidak menyentuh apa pun.")
    if not drv.simulate and input("Ketik YA untuk lanjut: ").strip().upper() != "YA":
        print("Dibatalkan.")
        return

    if not drv.arm(force=force_arm):
        print("Arm gagal - uji dibatalkan.")
        return
    try:
        for tid in range(1, NUM_THRUSTERS + 1):
            for sign, label in ((+1, "maju"), (-1, "mundur")):
                print(f"Thruster {tid}: {label} (level {level:.2f}) ...")
                _hold(drv, tid, sign * level, hold_s)
                drv.stop_all()
                time.sleep(1.0)
    finally:
        drv.stop_all()
        drv.disarm(force=force_arm)
    print("Selesai.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Driver thruster ROV")
    parser.add_argument("--simulate", action="store_true", help="mode simulasi (tanpa Pixhawk)")
    parser.add_argument("--port", help="port serial, mis. COM3 atau /dev/ttyACM0")
    parser.add_argument("--baud", type=int, help="baudrate")
    parser.add_argument("--bench", action="store_true", help="uji bangku tiap thruster (LEPAS PROPELLER)")
    parser.add_argument("--level", type=float, default=0.4, help="level uji 0..1 (dikali MAX_THRUST)")
    parser.add_argument("--hold", type=float, default=2.0, help="lama tiap uji (detik)")
    parser.add_argument("--force-arm", action="store_true", help="paksa arm (lewati preflight check)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    with ThrusterDriver(simulate=args.simulate or None, port=args.port, baud=args.baud) as drv:
        if args.bench:
            _bench_test(drv, args.level, args.hold, args.force_arm)
        else:
            print("Terhubung. Status:", drv.get_state())


if __name__ == "__main__":
    main()