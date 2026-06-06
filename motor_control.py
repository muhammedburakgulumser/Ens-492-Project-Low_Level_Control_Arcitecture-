"""
Motor Controller — Serial Monitor Modu
WiFi: ESP32_Motor_Net / password123

Komutlar:
  motor<n>            Aktif motoru seç (1-6)
  pos <derece>        Pozisyon gönder
  vel <rad/s>         Hız gönder
  kp / ki / kd <val>  PID güncelle
  ping                CAN bus durumu
  exit                Çıkış
"""

import requests
import threading
import time
import sys

ESP_IP   = "http://192.168.4.1"
TIMEOUT  = 0.8

# ── Paylaşılan durum ──────────────────────────────────────────────────────────
active_motor  = 1
running       = True
status_lock   = threading.Lock()

# Motor verisi (ESP32'den gelen)
motor_data = {
    i: {"tp": 0.0, "cp": 0.0, "tv": 0.0, "cv": 0.0,
        "kp": 0.0, "ki": 0.0, "kd": 0.0}
    for i in range(1, 7)
}

conn_error = ""   # bağlantı hata mesajı


# ── Fetch thread — 150ms'de bir veri çek, serial monitor gibi yazdır ─────────
def fetch_loop():
    global conn_error

    while running:
        m = active_motor
        try:
            r = requests.get(f"{ESP_IP}/status",
                             params={"m": m},
                             timeout=TIMEOUT)
            r.raise_for_status()
            d = r.json()

            with status_lock:
                motor_data[m].update(d)
                conn_error = ""

            # ── Serial monitordeki printContinuousStatus() ile birebir aynı format
            print(
                f"[M{m}] "
                f"TarPos: {d['tp']:.2f}° | "
                f"CurPos: {d['cp']:.2f}° | "
                f"TarVel: {d['tv']:.4f} rad/s | "
                f"CurVel: {d['cv']:.4f} rad/s | "
                f"Kp:{d['kp']:.2f} Ki:{d['ki']:.3f} Kd:{d['kd']:.3f}"
            )

        except Exception as e:
            with status_lock:
                conn_error = str(e)
            print(f"[NO CONNECTION] {conn_error}")

        time.sleep(0.15)   # ESP32 ile aynı 150ms cycle


# ── Komut gönder ──────────────────────────────────────────────────────────────
def send_cmd(motor_id: int, cmd_type: str, val: float):
    try:
        r = requests.get(
            f"{ESP_IP}/set",
            params={"m": motor_id, "type": cmd_type, "val": val},
            timeout=TIMEOUT
        )
        print(f"  >> [{r.text}]")
    except Exception as e:
        print(f"  >> Gönderim hatası: {e}")


# ── Komut parse ───────────────────────────────────────────────────────────────
def handle_command(cmd: str):
    global active_motor, running

    cmd = cmd.strip()
    if not cmd:
        return

    lo = cmd.lower()

    # --- exit
    if lo == "exit":
        running = False
        print("Çıkılıyor...")
        return

    # --- ping
    if lo == "ping":
        try:
            r = requests.get(f"{ESP_IP}/ping", timeout=TIMEOUT)
            print(f"\n  {r.text}\n")
        except Exception as e:
            print(f"\n  Ping başarısız: {e}\n")
        return

    # --- motor<n>
    if lo.startswith("motor"):
        try:
            mid = int(lo.replace("motor", ""))
            if 1 <= mid <= 6:
                active_motor = mid
                print(f"\n>>> SEÇİLEN MOTOR: M{active_motor}\n")
            else:
                print("  Motor ID 1-6 arasında olmalı.")
        except ValueError:
            print("  Geçersiz komut.")
        return

    # --- pos <derece>
    if lo.startswith("pos"):
        try:
            val = float(lo[3:].strip())
            send_cmd(active_motor, "pos", val)
        except ValueError:
            print("  Kullanım: pos <derece>   örn: pos 90")
        return

    # --- vel <rad/s>
    if lo.startswith("vel"):
        try:
            val = float(lo[3:].strip())
            send_cmd(active_motor, "vel", val)
        except ValueError:
            print("  Kullanım: vel <rad/s>   örn: vel 1.5")
        return

    # --- kp / ki / kd <val>
    for pid in ("kp", "ki", "kd"):
        if lo.startswith(pid):
            try:
                val = float(lo[2:].strip())
                send_cmd(active_motor, pid, val)
            except ValueError:
                print(f"  Kullanım: {pid} <değer>   örn: {pid} 0.30")
            return

    print(f"  Bilinmeyen komut: '{cmd}'")
    print("  Komutlar: motor<n> | pos | vel | kp | ki | kd | ping | exit")


# ── Ana döngü — input thread ayrı, fetch thread ayrı ─────────────────────────
def main():
    print(__doc__)
    print(f"Bağlanılıyor: {ESP_IP}  (WiFi: ESP32_Motor_Net)\n")

    # Fetch thread'i başlat
    t = threading.Thread(target=fetch_loop, daemon=True)
    t.start()

    # Ana thread sadece input okur — fetch thread yazdırmaya devam eder
    while running:
        try:
            cmd = input()
        except (EOFError, KeyboardInterrupt):
            print("\nÇıkılıyor.")
            break
        handle_command(cmd)


if __name__ == "__main__":
    main()
