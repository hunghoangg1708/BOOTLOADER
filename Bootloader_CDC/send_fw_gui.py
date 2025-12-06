# -*- coding: utf-8 -*-
"""
GUI nạp firmware qua USB CDC cho STM32 (Tkinter)
- Chọn file .bin
- Chọn COM (tự lọc VID/PID 0483:5740 nhưng có thể chọn cổng khác)
- Log hiển thị tiến trình + progress bar
Giữ nguyên giao thức như tool CLI trước đó của bạn.
"""

import sys, struct, time, threading, queue
from pathlib import Path

# --- serial / ports ---
import serial
import serial.tools.list_ports as stp

# --- tkinter UI ---
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

DEF_VID, DEF_PID = 0x0483, 0x5740  # ST CDC (Virtual COM)

# ------------- giao thức/tiện ích -------------
def list_ports_preferred():
    """Trả về ([(dev, label, is_st)], best_index) – cổng ST ưu tiên đầu danh sách."""
    items = []
    best = -1
    for p in stp.comports():
        label = f"{p.device} — {(p.manufacturer or '')} {(p.description or '')}".strip()
        is_st = (p.vid == DEF_VID and p.pid == DEF_PID) or \
                ('STMicro' in (p.manufacturer or '') or 'STM' in (p.description or ''))
        items.append((p.device, label, is_st))
    # Ưu tiên cổng ST first
    for i, (_, _, is_st) in enumerate(items):
        if is_st:
            best = i
            break
    return items, best

def read_line_safe(ser, timeout_ms=3000):
    """Đọc 1 dòng (kết thúc '\n'), bỏ '\r', an toàn cho timeout thay đổi cục bộ."""
    deadline = time.time() + timeout_ms / 1000.0
    buf = bytearray()
    while time.time() < deadline:
        try:
            b = ser.read(1)
        except serial.SerialException:
            break
        if not b:
            continue
        if b == b'\n':
            break
        buf += b
    return buf.decode(errors='ignore').strip('\r')

def wait_for_tag(ser, wanted, timeout_ms, log=None):
    """Đợi một trong các tag trong 'wanted' (set/list). Bỏ qua dòng rác."""
    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        line = read_line_safe(ser, timeout_ms=600)
        if not line:
            continue
        if log:
            log(f"[MCU] {line}")
        for tag in wanted:
            if tag in line:
                return line
    return ""

# ------------- Worker nạp FW -------------
class Flasher(threading.Thread):
    def __init__(self, port, baud, bin_path, log_cb, progress_cb, done_cb):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.bin_path = Path(bin_path)
        self.log = log_cb
        self.progress = progress_cb
        self.done = done_cb
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        try:
            data = self.bin_path.read_bytes()
        except Exception as e:
            self.log(f"[PC] Lỗi đọc file: {e}")
            self.done(False); return

        fw_size = len(data)
        fw_sum  = (sum(data) & 0xFFFFFFFF)
        self.log(f"[PC] File: {self.bin_path}  size={fw_size}  checksum=0x{fw_sum:08X}")

        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.10, write_timeout=1.0)
        except Exception as e:
            self.log(f"[PC] Không mở được cổng {self.port}: {e}")
            self.done(False); return

        try:
            # Một số driver cần DTR để “open port” theo nghĩa ứng dụng
            try: ser.setDTR(True)
            except Exception: pass
            time.sleep(0.3)

            # ---- Chờ BL_READY ----
            self.log("[PC] Chờ BL_READY...")
            tag = wait_for_tag(ser, {"BL_READY"}, 8000, self.log)
            if tag:
                self.log("[PC] Bootloader sẵn sàng.")
            else:
                self.log("[PC] Không thấy BL_READY — vẫn gửi header.")

            # Dọn input cũ (nếu còn RX_ERR từ lần trước)
            try: ser.reset_input_buffer()
            except Exception: pass

            # ---- GỬI HEADER ----
            header = struct.pack("<II", fw_size, fw_sum)
            ser.write(header); ser.flush()
            self.log("[PC] Đã gửi header.")

            # ---- CHỜ READY hoặc FW_SIZE_ERR ----
            tag = wait_for_tag(ser, {"READY", "FW_SIZE_ERR"}, 8000, self.log)
            if tag == "" or "READY" not in tag:
                if "FW_SIZE_ERR" in tag:
                    self.log("[PC] Dừng vì MCU báo FW_SIZE_ERR sau header.")
                else:
                    self.log("[PC] Không thấy READY – dừng để tránh tràn FIFO.")
                try: ser.close()
                except: pass
                self.done(False); return

            # Dọn input trước khi bơm dữ liệu
            try: ser.reset_input_buffer()
            except Exception: pass

            # ---- GỬI FIRMWARE ----
            CHUNK = 256
            sent = 0
            sum_stream = 0
            last_pct = -1

            for i in range(0, fw_size, CHUNK):
                if self._stop:
                    raise RuntimeError("Hủy bởi người dùng")
                chunk = data[i:i+CHUNK]
                ser.write(chunk)
                sent += len(chunk)
                sum_stream = (sum_stream + sum(chunk)) & 0xFFFFFFFF
                pct = sent*100//fw_size
                if pct != last_pct:
                    self.progress(pct/100.0)
                    last_pct = pct
                if (i//CHUNK) % 16 == 0 or sent == fw_size:
                    self.log(f"[PC] Sent {sent}/{fw_size} ({pct}%)")
                time.sleep(0.007)  # làm dịu host/stack USB

            ser.flush()
            self.log(f"[PC] SUM_STREAM=0x{sum_stream:08X}")

            # ---- Đợi phản hồi cuối (hoặc có thể BL nhảy app và tắt USB) ----
            self.log("[PC] Đợi phản hồi cuối từ MCU (FW_OK/CRC_FAIL/RX_ERR/SUM_...)...")
            tail = wait_for_tag(ser, {"FW_OK","CRC_FAIL","RX_ERR","SUM_FLASH","FW_SUM"}, 5000, self.log)
            if not tail:
                self.log("[PC] <no reply> (có thể BL đã nhảy App và tắt USB)")

            try: ser.close()
            except: pass

            self.progress(1.0)
            self.done(True)
        except Exception as e:
            try:
                ser.close()
            except Exception:
                pass
            self.log(f"[PC] Lỗi: {e}")
            self.done(False)

# ------------- UI -------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("STM32 CDC Flasher")
        self.geometry("720x520")

        self.log_q = queue.Queue()
        self.worker = None
        self.bin_path = None

        # Top controls
        frm = ttk.Frame(self); frm.pack(fill="x", padx=10, pady=10)

        ttk.Label(frm, text="Cổng COM:").grid(row=0, column=0, sticky="w")
        self.cbo_port = ttk.Combobox(frm, width=40, state="readonly")
        self.cbo_port.grid(row=0, column=1, sticky="we", padx=5)
        ttk.Button(frm, text="Làm mới", command=self.refresh_ports).grid(row=0, column=2, padx=5)

        ttk.Label(frm, text="Baud:").grid(row=0, column=3, sticky="e")
        self.ent_baud = ttk.Entry(frm, width=10)
        self.ent_baud.insert(0, "115200")
        self.ent_baud.grid(row=0, column=4, sticky="w", padx=5)

        frm2 = ttk.Frame(self); frm2.pack(fill="x", padx=10)
        self.lbl_file = ttk.Label(frm2, text="Chưa chọn file .bin")
        self.lbl_file.pack(side="left", fill="x", expand=True)
        ttk.Button(frm2, text="Chọn .bin…", command=self.pick_bin).pack(side="right", padx=5)

        # Progress
        frm3 = ttk.Frame(self); frm3.pack(fill="x", padx=10, pady=(8,4))
        self.prog = ttk.Progressbar(frm3, mode="determinate")
        self.prog.pack(fill="x", expand=True)
        self.prog["value"] = 0

        # Buttons
        frm4 = ttk.Frame(self); frm4.pack(fill="x", padx=10, pady=(4,8))
        self.btn_flash = ttk.Button(frm4, text="Nạp firmware", command=self.start_flash, state="disabled")
        self.btn_flash.pack(side="left")
        self.btn_stop  = ttk.Button(frm4, text="Hủy", command=self.stop_flash, state="disabled")
        self.btn_stop.pack(side="left", padx=6)

        # Log area
        self.txt = tk.Text(self, height=20)
        self.txt.pack(fill="both", expand=True, padx=10, pady=(0,10))
        self.txt.configure(font=("Consolas", 10))
        self.txt.tag_configure("mono", font=("Consolas", 10))

        self.refresh_ports()
        self.after(50, self._drain_log_queue)

    # ---- UI helpers ----
    def refresh_ports(self):
        items, best = list_ports_preferred()
        self.cbo_port["values"] = [f"{dev}  —  {label}" for dev, label, _ in items] if items else []
        if items:
            self.cbo_port.current(best if best != -1 else 0)
        else:
            self.cbo_port.set("")
        self._update_flash_button_state()

    def pick_bin(self):
        p = filedialog.askopenfilename(
            title="Chọn firmware (.bin)",
            filetypes=[("Binary firmware", "*.bin"), ("All files", "*.*")]
        )
        if p:
            self.bin_path = Path(p)
            try:
                size = self.bin_path.stat().st_size
                self.lbl_file.config(text=f"{self.bin_path}  ({size} bytes)")
            except Exception:
                self.lbl_file.config(text=str(self.bin_path))
        self._update_flash_button_state()

    def _update_flash_button_state(self):
        enable = bool(self.cbo_port.get()) and bool(self.bin_path)
        self.btn_flash.config(state="normal" if enable and not self.worker else "disabled")

    def log(self, s):
        self.log_q.put(str(s))

    def _drain_log_queue(self):
        try:
            while True:
                s = self.log_q.get_nowait()
                self.txt.insert("end", s + "\n", "mono")
                self.txt.see("end")
        except queue.Empty:
            pass
        self.after(50, self._drain_log_queue)

    def set_progress(self, f):
        f = max(0.0, min(1.0, float(f)))
        self.prog["value"] = int(f * 100)

    # ---- Flash control ----
    def start_flash(self):
        if self.worker:
            return
        if not self.bin_path:
            messagebox.showwarning("Thiếu file", "Chưa chọn file .bin")
            return
        port_display = self.cbo_port.get()
        if not port_display:
            messagebox.showwarning("Thiếu cổng", "Chưa chọn cổng COM")
            return
        port = port_display.split()[0]  # lấy "COMx" từ chuỗi

        try:
            baud = int(self.ent_baud.get().strip())
        except Exception:
            messagebox.showerror("Baud sai", "Giá trị baud không hợp lệ.")
            return

        self.txt.delete("1.0", "end")
        self.set_progress(0)
        self.btn_flash.config(state="disabled")
        self.btn_stop.config(state="normal")

        self.worker = Flasher(
            port=port,
            baud=baud,
            bin_path=self.bin_path,
            log_cb=self.log,
            progress_cb=self.set_progress,
            done_cb=self._flash_done
        )
        self.worker.start()

    def stop_flash(self):
        if self.worker:
            self.worker.stop()
            self.log("[PC] Yêu cầu dừng…")

    def _flash_done(self, ok):
        # callback từ thread (được gọi trong thread) -> chuyển về main thread
        def _finish():
            self.btn_stop.config(state="disabled")
            self.worker = None
            self._update_flash_button_state()
            if ok:
                self.log("[PC] Hoàn tất.")
            else:
                self.log("[PC] Thất bại.")
        self.after(0, _finish)

# ------------- main -------------
if __name__ == "__main__":
    try:
        App().mainloop()
    except KeyboardInterrupt:
        sys.exit(1)