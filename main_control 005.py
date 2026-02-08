#!/usr/bin/env python3
"""
Kontrollprogramm Aufzug
A. Bösmann
CC-BY
v0.01

#!/usr/bin/env python3

main_test.py - PyQt5 GUI to control the Aufzug motor MCU

Protocol with MCU (Serial, line-based):
- Send: `SET PPR 2000`, `SET WD 5.0`, `SET SPEED 1.2`
- Send: `BUTTON UP`, `BUTTON DOWN`, `STOP`, `STATUS`, `PING`
- Receive: `ACK:...`, `TICK:elapsed_ms,speed`, `DONE:MOVE`, `ERROR:...`

This GUI is non-blocking: serial I/O runs in a background thread.
Saves tick data to CSV after each move.
"""

import sys
import csv
from datetime import datetime
import os
import json
from PyQt5 import QtWidgets, QtCore
from typing import Optional, cast
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import socket
import random
import time as _time
from typing import Dict
from collections import deque


class ExperimentWindow(QtWidgets.QDialog):
    def __init__(self, main_win):
        super().__init__(main_win)
        self.main = main_win
        self.setWindowTitle('Experiment')
        self.resize(420, 220)
        v = QtWidgets.QVBoxLayout(self)

        self.lbl_status = QtWidgets.QLabel('Idle')
        self.vt_label = QtWidgets.QLabel('VTable: unknown')
        self.vt_label.setStyleSheet('color: gray')
        self.lbl_elapsed = QtWidgets.QLabel('Elapsed: --')
        self.lbl_speed = QtWidgets.QLabel('Speed: --')
        self.lbl_pwm = QtWidgets.QLabel('PWM: --')
        self.lbl_dir = QtWidgets.QLabel('Dir: --')
        self.lbl_ts = QtWidgets.QLabel('TS(us): --')
        v.addWidget(self.lbl_status)
        v.addWidget(self.lbl_elapsed)
        v.addWidget(self.lbl_speed)
        v.addWidget(self.lbl_pwm)
        v.addWidget(self.lbl_dir)
        v.addWidget(self.lbl_ts)

        h = QtWidgets.QHBoxLayout()
        h.addWidget(QtWidgets.QLabel('Repetitions'))
        self.reps_spin = QtWidgets.QSpinBox()
        self.reps_spin.setRange(1, 1000)
        self.reps_spin.setValue(1)
        h.addWidget(self.reps_spin)
        h.addSpacing(10)
        h.addWidget(QtWidgets.QLabel('Duration (s)'))
        self.duration_spin = QtWidgets.QSpinBox()
        self.duration_spin.setRange(1, 3600)
        # initialize from main window's duration if available
        try:
            self.duration_spin.setValue(int(self.main.duration_spin.value()))
        except Exception:
            self.duration_spin.setValue(1)
        # place duration input next to its label
        h.addWidget(self.duration_spin)
        h.addSpacing(10)
        h.addWidget(QtWidgets.QLabel('Time step (us)'))
        self.time_step_spin = QtWidgets.QSpinBox()
        self.time_step_spin.setRange(100, 1000000)  # 0.1 ms .. 1 s
        self.time_step_spin.setSingleStep(100)
        self.time_step_spin.setValue(1000)  # default 1 ms
        h.addWidget(self.time_step_spin)
        v.addLayout(h)

        btn_h = QtWidgets.QHBoxLayout()
        self.start_btn = QtWidgets.QPushButton('Start')
        self.stop_btn = QtWidgets.QPushButton('Stop')
        self.save_btn = QtWidgets.QPushButton('Save Results')
        self.check_vt_btn = QtWidgets.QPushButton('Check VTable')
        self.check_vt_btn.setToolTip('Query MCU whether a movement V_TABLE is present')
        btn_h.addWidget(self.start_btn)
        btn_h.addWidget(self.stop_btn)
        btn_h.addWidget(self.save_btn)
        btn_h.addWidget(self.check_vt_btn)
        v.addLayout(btn_h)

        self.start_btn.clicked.connect(self.on_start)
        self.stop_btn.clicked.connect(self.on_stop)
        self.save_btn.clicked.connect(self.on_save)
        self.check_vt_btn.clicked.connect(self.check_vtable)

        self.results = []
        self.running = False
        self.reps_total = 1
        # flag to indicate we requested a VTable check on window open
        self._checked_on_open = False

    def on_start(self):
        if not self.main.serial_thread:
            QtWidgets.QMessageBox.warning(self, 'Not connected', 'Please connect to MCU')
            return
        self.results = []
        self.reps_total = int(self.reps_spin.value())
        # transmit repetitions to MCU; MCU handles repetitions if implemented
        try:
            # inform MCU about desired run duration (seconds)
            try:
                dur = int(self.duration_spin.value())
                self.main.send_command(f'SET DURATION {dur}')
            except Exception:
                pass
            # inform MCU about desired time step in microseconds
            try:
                ts = int(self.time_step_spin.value())
                self.main.send_command(f'SET TIME_STEP_US {ts}')
            except Exception:
                pass
            self.main.send_command(f'REPS {self.reps_total}')
        except Exception:
            pass
        try:
            self.main.send_settings(include_duration=False)
            self.main.send_command('START_EXPERIMENT')
        except Exception:
            pass
        self.running = True
        self.lbl_status.setText(f'Running (1/{self.reps_total})')
        # Update vtable status when starting
        try:
            self.check_vtable()
        except Exception:
            pass

    def on_stop(self):
        try:
            self.main.send_command('STOP_EXPERIMENT')
        except Exception:
            pass
        self.running = False
        self.lbl_status.setText('Stopped')

    def on_save(self):
        if not self.results:
            QtWidgets.QMessageBox.information(self, 'No data', 'No experiment data to save')
            return
        fname, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Save Experiment results', '', 'CSV Files (*.csv);;All Files (*)')
        if not fname:
            return
        try:
            with open(fname, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['idx','elapsed_ms','pwm','dir','target_speed'])
                for r in self.results:
                    writer.writerow([r.get('idx'), r.get('elapsed_ms'), r.get('pwm'), r.get('dir'), r.get('target_speed')])
            QtWidgets.QMessageBox.information(self, 'Saved', f'Saved {fname}')
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Save failed', f'Failed to save results: {e}')

    def append_tick(self, elapsed_ms, speed):
        self.lbl_elapsed.setText(f'Elapsed: {elapsed_ms} ms')
        self.lbl_speed.setText(f'Speed: {speed} cm/s')
        self.results.append({'idx': None, 'elapsed_ms': elapsed_ms, 'pwm': None, 'dir': None, 'target_speed': speed})

    def append_dbg_update(self, fields: dict):
        # parse fields with local temporaries so type-checker can reason about None
        idx = None
        try:
            v = fields.get('IDX')
            if v is not None and v != '':
                idx = int(v)
        except Exception:
            idx = None

        elapsed = None
        try:
            v = fields.get('ELAPSED')
            if v is not None and v != '':
                elapsed = int(float(v))
        except Exception:
            elapsed = None

        # prefer GLOBAL_US (microseconds since experiment start) if provided
        global_us = None
        try:
            v = fields.get('GLOBAL_US')
            if v is not None and v != '':
                global_us = int(v)
        except Exception:
            global_us = None
        if global_us is not None:
            try:
                elapsed = int(global_us // 1000)
            except Exception:
                pass

        tgt = None
        try:
            v = fields.get('TGT')
            if v is not None and v != '':
                tgt = float(v)
        except Exception:
            tgt = None

        pwm = None
        try:
            v = fields.get('PWM')
            if v is not None and v != '':
                pwm = float(v)
        except Exception:
            pwm = None

        dirv = fields.get('DIR')
        row = {'idx': idx, 'elapsed_ms': elapsed, 'pwm': pwm, 'dir': dirv, 'target_speed': tgt}
        if global_us is not None:
            row['global_us'] = global_us
        # parse timestamp in micros if provided
        ts_us = None
        try:
            v = fields.get('TS_US')
            if v is not None and v != '':
                ts_us = int(v)
            else:
                v2 = fields.get('US')
                if v2 is not None and v2 != '':
                    ts_us = int(v2)
        except Exception:
            ts_us = None
        if ts_us is not None:
            row['ts_us'] = ts_us
            try:
                self.lbl_ts.setText(f'TS(us): {ts_us}')
            except Exception:
                pass
        self.results.append(row)
        if elapsed is not None:
            self.lbl_elapsed.setText(f'Elapsed: {elapsed} ms')
        if tgt is not None:
            self.lbl_speed.setText(f'Speed: {tgt:.1f} cm/s')
        if pwm is not None:
            self.lbl_pwm.setText(f'PWM: {pwm:.0f}')
        if dirv is not None:
            self.lbl_dir.setText(f'Dir: {dirv}')

    def check_vtable(self):
        # ask MCU whether V_TABLE is present
        if not self.main.serial_thread:
            # show unknown in gray
            try:
                self.vt_label.setText('VTable: unknown')
                self.vt_label.setStyleSheet('color: gray')
            except Exception:
                pass
            return
        try:
            self.main.send_command('GET_VTABLE')
        except Exception:
            pass

    def showEvent(self, a0):
        # when the dialog is shown, proactively query MCU for V_TABLE
        try:
            self._checked_on_open = True
            # give main window a moment if not connected yet
            QtCore.QTimer.singleShot(50, self.check_vtable)
        except Exception:
            pass
            super().showEvent(a0)

    def update_vtable_status(self, length: int):
        try:
            if length and length > 0:
                self.vt_label.setText(f'VTable: present ({length} rows)')
                self.vt_label.setStyleSheet('background-color: green; color: white;')
            else:
                self.vt_label.setText('VTable: missing')
                self.vt_label.setStyleSheet('background-color: red; color: white;')
                # if this was the automatic check triggered on open, notify the user
                try:
                    if getattr(self, '_checked_on_open', False):
                        QtWidgets.QMessageBox.warning(self, 'No movement data', 'No movement V_TABLE present on MCU. Please upload movement data before starting the experiment.')
                except Exception:
                    pass
            # clear the one-shot open-check flag after handling
            try:
                self._checked_on_open = False
            except Exception:
                pass
        except Exception:
            pass

    def append_restart(self, left: int):
        # normalize input
        try:
            left_i = int(left)
        except Exception:
            return

        # ensure reps_total is an int (may be None or a string)
        try:
            rt = int(self.reps_total) if getattr(self, 'reps_total', None) is not None else 0
        except Exception:
            rt = 0

        if rt > 0:
            run_idx = (rt - left_i) + 1
            if run_idx < 1:
                run_idx = 1
            if run_idx > rt:
                run_idx = rt
            try:
                self.lbl_status.setText(f'Running ({run_idx}/{rt})')
            except Exception:
                pass
        else:
            try:
                self.lbl_status.setText(f'Running (next, left={left_i})')
            except Exception:
                pass

        # keep running flag true while experiment continues
        self.running = True

    def handle_done_experiment(self, reason: Optional[str] = None):
        if not self.running:
            if reason:
                self.lbl_status.setText(f'Finished ({reason})')
            return
        self.running = False
        if reason:
            self.lbl_status.setText(f'Finished ({reason})')
        else:
            self.lbl_status.setText('Finished')

    def closeEvent(self, a0):
        try:
            self.main.exp_window = None
        except Exception:
            pass
        super().closeEvent(a0)


class CalibrationWindow(QtWidgets.QDialog):
    def __init__(self, main_win):
        super().__init__(main_win)
        self.main = main_win
        self.setWindowTitle('Calibration')
        self.resize(360, 220)
        v = QtWidgets.QVBoxLayout(self)

        # Current sensor height display
        h1 = QtWidgets.QHBoxLayout()
        h1.addWidget(QtWidgets.QLabel('Sensor height (cm):'))
        self.lbl_height = QtWidgets.QLabel('---')
        h1.addWidget(self.lbl_height)
        v.addLayout(h1)

        # Buttons: Up, Down, Set Zero
        btn_h = QtWidgets.QHBoxLayout()
        self.up_btn = QtWidgets.QPushButton('Up')
        self.down_btn = QtWidgets.QPushButton('Down')
        self.setzero_btn = QtWidgets.QPushButton('Set Zero')
        self.save_calib_btn = QtWidgets.QPushButton('Save Calibration')
        btn_h.addWidget(self.up_btn)
        btn_h.addWidget(self.down_btn)
        btn_h.addWidget(self.setzero_btn)
        btn_h.addWidget(self.save_calib_btn)
        v.addLayout(btn_h)

        # Set length input and pulses display
        h2 = QtWidgets.QHBoxLayout()
        h2.addWidget(QtWidgets.QLabel('Set length (cm):'))
        self.set_length_spin = QtWidgets.QDoubleSpinBox()
        self.set_length_spin.setRange(10.0, 400.0)
        self.set_length_spin.setDecimals(2)
        self.set_length_spin.setValue(100.0)
        h2.addWidget(self.set_length_spin)
        h2.addWidget(QtWidgets.QLabel('Pulses:'))
        self.lbl_pulses = QtWidgets.QLabel('0')
        h2.addWidget(self.lbl_pulses)
        v.addLayout(h2)

        # Measured length and correction factor
        h3 = QtWidgets.QHBoxLayout()
        h3.addWidget(QtWidgets.QLabel('Measured length (cm):'))
        self.measured_spin = QtWidgets.QDoubleSpinBox()
        self.measured_spin.setRange(0.0, 1000.0)
        self.measured_spin.setDecimals(2)
        self.measured_spin.setValue(0.0)
        h3.addWidget(self.measured_spin)
        h3.addWidget(QtWidgets.QLabel('Correction:'))
        self.lbl_correction = QtWidgets.QLabel('1.000')
        h3.addWidget(self.lbl_correction)
        v.addLayout(h3)

        # Hook up signals
        self.up_btn.clicked.connect(self.on_up)
        self.down_btn.clicked.connect(self.on_down)
        self.setzero_btn.clicked.connect(self.on_set_zero)
        self.set_length_spin.valueChanged.connect(self.update_pulses)
        self.measured_spin.valueChanged.connect(self.update_correction)
        self.save_calib_btn.clicked.connect(self.on_save_calibration)

        # internal state
        self._raw_height = 0.0
        self._zero_offset = 0.0
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._on_timer)
        self._timer.start(200)

        # if no real sensor available, simulate small changes
        self._simulate = (self.main.serial_thread is None)
        self._sim_value = 0.0

        # initial calculations
        self.update_pulses()
        self.update_correction()

    def _on_timer(self):
        # request sensor if possible
        if self.main.serial_thread:
            try:
                # request sensor reading (MCU may not implement; harmless)
                self.main.send_command('GET_SENSOR')
            except Exception:
                pass
        else:
            # simulate sensor value oscillation
            self._sim_value += 0.2
            if self._sim_value > 200.0:
                self._sim_value = 0.0
            self._raw_height = self._sim_value
            self._update_displayed_height()

    def update_sensor_height(self, h_cm: float):
        try:
            self._raw_height = float(h_cm)
        except Exception:
            return
        self._update_displayed_height()

    def _update_displayed_height(self):
        try:
            disp = self._raw_height - self._zero_offset
            self.lbl_height.setText(f'{disp:.2f}')
        except Exception:
            pass

    def update_pulses(self):
        # pulses = set_length / circumference * ppr
        try:
            length = float(self.set_length_spin.value())
            ppr = int(self.main.ppr_combo.currentText()) if self.main.ppr_combo.currentText().isdigit() else int(self.main.ppr_combo.currentText() or 0)
            wd = float(self.main.wd_spin.value())
            if wd <= 0 or ppr <= 0:
                self.lbl_pulses.setText('0')
                return
            circ = 3.141592653589793 * wd
            pulses = (length / circ) * ppr
            self.lbl_pulses.setText(str(int(round(pulses))))
        except Exception:
            self.lbl_pulses.setText('0')

    def update_correction(self):
        try:
            set_len = float(self.set_length_spin.value())
            meas = float(self.measured_spin.value())
            if set_len == 0:
                corr = 1.0
            else:
                corr = (meas / set_len) if set_len != 0 else 1.0
            self.lbl_correction.setText(f'{corr:.4f}')
        except Exception:
            self.lbl_correction.setText('1.0000')

    def on_set_zero(self):
        # set current raw height as zero offset
        try:
            self._zero_offset = float(self._raw_height)
            self._update_displayed_height()
        except Exception:
            pass

    def on_up(self):
        # move motor up by set_length: compute required duration from speed
        try:
            length = float(self.set_length_spin.value())
            speed = float(self.main.speed_spin.value())
            if speed <= 0.0:
                QtWidgets.QMessageBox.warning(self, 'Invalid speed', 'Set a non-zero speed in main window')
                return
            dur_s = length / speed
            # send duration and trigger BUTTON UP
            self.main.send_command(f'SET DURATION {int(max(1, round(dur_s)))}')
            # ensure current speed is sent
            self.main.send_settings(include_duration=True)
            self.main.send_command('BUTTON UP')
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Error', f'Failed to command motor: {e}')

    def on_down(self):
        try:
            length = float(self.set_length_spin.value())
            speed = float(self.main.speed_spin.value())
            if speed <= 0.0:
                QtWidgets.QMessageBox.warning(self, 'Invalid speed', 'Set a non-zero speed in main window')
                return
            dur_s = length / speed
            self.main.send_command(f'SET DURATION {int(max(1, round(dur_s)))}')
            self.main.send_settings(include_duration=True)
            self.main.send_command('BUTTON DOWN')
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Error', f'Failed to command motor: {e}')

    def closeEvent(self, a0):
        try:
            self._timer.stop()
        except Exception:
            pass
        try:
            self.main.calib_window = None
        except Exception:
            pass
        super().closeEvent(a0)

    def on_save_calibration(self):
        try:
            # gather calibration data
            cfg = {
                'ppr': int(self.main.ppr_combo.currentText()) if self.main.ppr_combo.currentText().isdigit() else None,
                'wheel_diameter_cm': float(self.main.wd_spin.value()),
                'set_length_cm': float(self.set_length_spin.value()),
                'measured_length_cm': float(self.measured_spin.value()),
                'correction_factor': float(self.lbl_correction.text()) if self.lbl_correction.text() else None
            }
            path = self.main._config_std_path()
            base = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
            out = os.path.join(base, 'calibration.json')
            with open(out, 'w') as f:
                json.dump(cfg, f, indent=2)
            QtWidgets.QMessageBox.information(self, 'Saved', f'Saved calibration to {out}')
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Save failed', f'Failed to save calibration: {e}')


class SerialThread(QtCore.QThread):
    line_received = QtCore.pyqtSignal(str)

    def __init__(self, port, baud=115200, parent=None): # 28800, 57600, 115200, 230400, 460800, 921600
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._running = True
        self._ser = None
        self._buffer = bytearray()

    def run(self):
        try:
            # open in non-blocking mode (timeout=0) and clea# r any existing input
            print(f'Opening serial port {self.port} at {self.baud} baud...')
            self._ser = serial.Serial(self.port, self.baud, timeout=0)
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
        except Exception as e:
            self.line_received.emit(f"ERROR:SERIAL_OPEN:{e}")
            return
        while self._running:
            try:
                # read only when data is available (non-blocking)
                to_read = self._ser.in_waiting
                if not to_read:                         
                    continue
                data = self._ser.read(to_read)
                if data:
                    self._buffer.extend(data)
                    # extract lines separated by LF efficiently; handle optional CR
                    while True:
                        nl = self._buffer.find(b'\n')
                        if nl == -1:
                            break
                        line_bytes = bytes(self._buffer[:nl])
                        # remove trailing CR if present
                        if line_bytes.endswith(b'\r'):
                            line_bytes = line_bytes[:-1]
                        try:
                            line = line_bytes.decode('utf-8', errors='ignore').strip()
                        except Exception:
                            line = ''
                        if line:
                            self.line_received.emit(line)
                        # remove processed bytes (+LF)
                        del self._buffer[:nl+1]
                else:
                    continue
            except Exception as e:
                self.line_received.emit(f"ERROR:SERIAL_READ:{e}")
                break

    def write_line(self, s: str):
        if self._ser and self._ser.is_open:
            self._ser.write((s + '\n').encode('utf-8'))

    def close(self):
        self._running = False
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass


class SensorThread(QtCore.QThread):
    measurement = QtCore.pyqtSignal(dict)

    def __init__(self, ip: str, port: int, meas_step_us: int, sensors: Dict[str, bool], parent=None):
        super().__init__(parent)
        self.ip = ip
        self.port = port
        self.meas_step_us = max(100, int(meas_step_us))
        self.sensors = sensors
        self._running = True
        self._sock = None
        self._file = None
        self._outfile_path = None

    def run(self):
        # try TCP connect; if fails, operate in simulated mode
        connected = False
        try:
            self._sock = socket.create_connection((self.ip, int(self.port)), timeout=0.5)
            self._sock.settimeout(0.2)
            connected = True
        except Exception:
            connected = False

        # open output file (timestamped)
        try:
            ts = datetime.now().strftime('sensors_%Y%m%d_%H%M%S.csv')
            self._outfile_path = ts
            self._file = open(ts, 'w', newline='')
            hdr = ['ts_us']
            for k, v in self.sensors.items():
                if v:
                    if k == 'IMU_ACCEL':
                        hdr.extend(['IMU_AX', 'IMU_AY', 'IMU_AZ'])
                    else:
                        hdr.append(k)
            self._file.write(','.join(hdr) + '\n')
        except Exception:
            self._file = None

        next_time = _time.time()
        while self._running:
            t0 = _time.time()
            # gather measurement
            meas = {'ts_us': int(_time.time() * 1e6)}
            if connected and self._sock:
                try:
                    # request measurement; protocol: send 'MEAS' and expect CSV line or JSON (best-effort)
                    try:
                        self._sock.sendall(b'MEAS\n')
                    except Exception:
                        pass
                    data = b''
                    try:
                        data = self._sock.recv(1024)
                    except Exception:
                        data = b''
                    if data:
                        try:
                            s = data.decode('utf-8', errors='ignore').strip()
                            # try parse csv: name=val,name=val
                            parts = s.split(',')
                            for p in parts:
                                if '=' in p:
                                    k, v = p.split('=', 1)
                                    meas[k.strip()] = float(v.strip()) if v.strip() != '' else None
                        except Exception:
                            pass
                except Exception:
                    pass
            else:
                # simulated readings
                if self.sensors.get('LIDAR'):
                    meas['LIDAR'] = round(100.0 + random.uniform(-2.0, 2.0), 2)
                if self.sensors.get('IMU_ACCEL'):
                    meas['IMU_AX'] = round(random.uniform(-0.2, 0.2), 3)
                    meas['IMU_AY'] = round(random.uniform(-0.2, 0.2), 3)
                    meas['IMU_AZ'] = round(9.81 + random.uniform(-0.2, 0.2), 3)
                if self.sensors.get('CONDUCTIVITY'):
                    meas['COND_V'] = round(random.uniform(0.1, 2.0), 3)

            # write to file if open
            if self._file:
                try:
                    row = [str(meas.get('ts_us', ''))]
                    for k, v in self.sensors.items():
                        if v:
                            if k == 'IMU_ACCEL':
                                row.append(str(meas.get('IMU_AX', '')))
                                row.append(str(meas.get('IMU_AY', '')))
                                row.append(str(meas.get('IMU_AZ', '')))
                            else:
                                row.append(str(meas.get(k, '')))
                    self._file.write(','.join(row) + '\n')
                except Exception:
                    pass

            # emit measurement for UI plotting
            try:
                self.measurement.emit(meas)
            except Exception:
                pass

            # sleep until next period
            period_s = self.meas_step_us / 1e6
            t1 = _time.time()
            to_sleep = period_s - (t1 - t0)
            if to_sleep > 0:
                _time.sleep(to_sleep)

        # cleanup
        try:
            if self._file:
                self._file.close()
        except Exception:
            pass


class SerialSensorThread(QtCore.QThread):
    measurement = QtCore.pyqtSignal(dict)

    def __init__(self, port: str, baud: int, meas_step_us: int, sensors: Dict[str, bool], parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = int(baud)
        self.meas_step_us = max(100, int(meas_step_us))
        self.sensors = sensors
        self._running = True
        self._ser = None
        self._file = None
        self._outfile_path = None

    def run(self):
        # try open serial port
        connected = False
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=1)
            connected = True
        except Exception:
            connected = False

        # open output file (timestamped)
        try:
            ts = datetime.now().strftime('sensors_serial_%Y%m%d_%H%M%S.csv')
            self._outfile_path = ts
            self._file = open(ts, 'w', newline='')
            hdr = ['ts_us']
            for k, v in self.sensors.items():
                if v:
                    if k == 'IMU_ACCEL':
                        hdr.extend(['IMU_AX', 'IMU_AY', 'IMU_AZ'])
                    else:
                        hdr.append(k)
            self._file.write(','.join(hdr) + '\n')
        except Exception:
            self._file = None

        while self._running:
            t0 = _time.time()
            meas = {'ts_us': int(_time.time() * 1e6)}
            if connected and self._ser:
                try:
                    line = self._ser.readline()
                    if line:
                        try:
                            s = line.decode('utf-8', errors='ignore').strip()
                        except Exception:
                            try:
                                s = str(line).strip()
                            except Exception:
                                s = ''
                        if s:
                            # parse csv: name=val,name=val
                            parts = s.split(',')
                            for p in parts:
                                if '=' in p:
                                    k, v = p.split('=', 1)
                                    try:
                                        meas[k.strip()] = float(v.strip()) if v.strip() != '' else None
                                    except Exception:
                                        meas[k.strip()] = v.strip()
                except Exception:
                    pass
            else:
                # simulated
                if self.sensors.get('LIDAR'):
                    meas['LIDAR'] = round(100.0 + random.uniform(-2.0, 2.0), 2)
                if self.sensors.get('IMU_ACCEL'):
                    meas['IMU_AX'] = round(random.uniform(-0.2, 0.2), 3)
                    meas['IMU_AY'] = round(random.uniform(-0.2, 0.2), 3)
                    meas['IMU_AZ'] = round(9.81 + random.uniform(-0.2, 0.2), 3)
                if self.sensors.get('CONDUCTIVITY'):
                    meas['COND_V'] = round(random.uniform(0.1, 2.0), 3)

            # write to file if open
            if self._file:
                try:
                    row = [str(meas.get('ts_us', ''))]
                    for k, v in self.sensors.items():
                        if v:
                            if k == 'IMU_ACCEL':
                                row.append(str(meas.get('IMU_AX', '')))
                                row.append(str(meas.get('IMU_AY', '')))
                                row.append(str(meas.get('IMU_AZ', '')))
                            else:
                                row.append(str(meas.get(k, '')))
                    self._file.write(','.join(row) + '\n')
                except Exception:
                    pass

            try:
                self.measurement.emit(meas)
            except Exception:
                pass

            period_s = self.meas_step_us / 1e6
            t1 = _time.time()
            to_sleep = period_s - (t1 - t0)
            if to_sleep > 0:
                _time.sleep(to_sleep)

        try:
            if self._file:
                self._file.close()
        except Exception:
            pass
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass

    def stop(self):
        self._running = False


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Aufzug')
        self.resize(1400, 800)
        # typed attributes for static analysis
        self.serial_thread: Optional[SerialThread] = None
        self.exp_window: Optional[ExperimentWindow] = None
        self.calib_window: Optional[CalibrationWindow] = None
        self.log_lines = []
        self.ticks = []

        # Central widget and layout
        central = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(central)
        self.setCentralWidget(central)

        # Menu bar
        menubar = cast(QtWidgets.QMenuBar, self.menuBar())
        # File menu
        file_menu = cast(QtWidgets.QMenu, menubar.addMenu('File'))
        act_open_movement = QtWidgets.QAction('Open Movement data', self)
        act_open_movement.triggered.connect(self.open_movement_data)
        file_menu.addAction(act_open_movement)
        act_open_conf = QtWidgets.QAction('Open Configuration', self)
        act_open_conf.triggered.connect(self.open_configuration)
        file_menu.addAction(act_open_conf)
        act_save_results = QtWidgets.QAction('Save Experiment results', self)
        act_save_results.setShortcut('Ctrl+S')
        act_save_results.triggered.connect(self.save_experiment_results)
        file_menu.addAction(act_save_results)
        file_menu.addSeparator()
        act_quit = QtWidgets.QAction('Quit', self)
        act_quit.setShortcut('Ctrl+Q')
        # define a simple slot wrapper that returns None (satisfies PyQt slot typing)
        def _close_slot() -> None:
            self.close()
        act_quit.triggered.connect(_close_slot)
        file_menu.addAction(act_quit)

        # Calibration menu
        calib_menu = cast(QtWidgets.QMenu, menubar.addMenu('Calibration'))
        act_load_calib = QtWidgets.QAction('Load Calibration', self)
        act_load_calib.triggered.connect(self.load_calibration)
        calib_menu.addAction(act_load_calib)
        act_start_calib = QtWidgets.QAction('Start Calibration', self)
        act_start_calib.triggered.connect(self.start_calibration)
        calib_menu.addAction(act_start_calib)
        act_motor_data = QtWidgets.QAction('Motor data', self)
        act_motor_data.triggered.connect(self.motor_data)
        calib_menu.addAction(act_motor_data)

        # Experiment menu
        exp_menu = cast(QtWidgets.QMenu, menubar.addMenu('Experiment'))
        act_start_exp = QtWidgets.QAction('Start Experiment', self)
        act_start_exp.triggered.connect(self.start_experiment)
        exp_menu.addAction(act_start_exp)

        # Help menu
        help_menu = cast(QtWidgets.QMenu, menubar.addMenu('Help'))
        act_help = QtWidgets.QAction('Help', self)
        act_help.triggered.connect(self.show_help)
        help_menu.addAction(act_help)
        act_about = QtWidgets.QAction('About', self)
        act_about.triggered.connect(self.show_about)
        help_menu.addAction(act_about)

        # Status bar (keep typed reference to satisfy type checker)
        self._status_bar = cast(QtWidgets.QStatusBar, self.statusBar())
        self._status_bar.showMessage('Ready')

        # Widgets
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports()
        self.connect_btn = QtWidgets.QPushButton('Connect')
        self.connect_btn.clicked.connect(self.toggle_connect)
        self.ppr_combo = QtWidgets.QComboBox()
        for v in [800,1000,1600,2000,3200,4000,5000,6400,8000,10000,12800,20000,25600,40000,51200]:
            self.ppr_combo.addItem(str(v))
        self.wd_spin = QtWidgets.QDoubleSpinBox(); self.wd_spin.setRange(1.0,100.0); self.wd_spin.setDecimals(2); self.wd_spin.setValue(5.0)
        self.speed_spin = QtWidgets.QDoubleSpinBox(); self.speed_spin.setRange(0.02,50.0); self.speed_spin.setDecimals(3); self.speed_spin.setValue(1.0)
        self.send_settings_btn = QtWidgets.QPushButton('Send Settings')
        self.send_settings_btn.clicked.connect(self.send_settings)
        self.load_cfg_btn = QtWidgets.QPushButton('Load config')
        self.load_cfg_btn.clicked.connect(self.load_std_config)
        self.save_cfg_btn = QtWidgets.QPushButton('Save config')
        self.save_cfg_btn.clicked.connect(self.save_std_config)
        self.clear_log_btn = QtWidgets.QPushButton('Clear Log')
        self.clear_log_btn.clicked.connect(self.clear_log)
        self.duration_spin = QtWidgets.QSpinBox(); self.duration_spin.setRange(1,30); self.duration_spin.setValue(1)
        # Sensor controls
        grid.addWidget(QtWidgets.QLabel('Sensor IP'), 0, 3)
        self.sensor_ip = QtWidgets.QLineEdit('10.188.1.101')
        grid.addWidget(self.sensor_ip, 0, 4)
        grid.addWidget(QtWidgets.QLabel('Sensor Port'), 1, 3)
        self.sensor_port = QtWidgets.QSpinBox(); self.sensor_port.setRange(1,65535); self.sensor_port.setValue(5000)
        grid.addWidget(self.sensor_port, 1, 4)
        # add serial port selection for sensors (USB/COM)
        grid.addWidget(QtWidgets.QLabel('Sensor COM'), 0, 5)
        self.sensor_port_combo = QtWidgets.QComboBox()
        self.sensor_port_combo.setEditable(False)
        grid.addWidget(self.sensor_port_combo, 0, 6)
        self.sensor_refresh_btn = QtWidgets.QPushButton('Refresh')
        self.sensor_refresh_btn.clicked.connect(lambda: self._refresh_sensor_ports())
        grid.addWidget(self.sensor_refresh_btn, 1, 6)
        self.sensor_detect_btn = QtWidgets.QPushButton('Detect Teensy')
        self.sensor_detect_btn.clicked.connect(lambda: self._detect_teensy())
        grid.addWidget(self.sensor_detect_btn, 2, 6)
        self.sensor_connect_btn = QtWidgets.QPushButton('Connect Sensors')
        self.sensor_connect_btn.clicked.connect(self.toggle_sensor_connect)
        grid.addWidget(self.sensor_connect_btn, 2, 3, 1, 2)
        self.sensor_start_btn = QtWidgets.QPushButton('Start Sensors')
        self.sensor_start_btn.clicked.connect(self.toggle_sensor_sampling)
        grid.addWidget(self.sensor_start_btn, 2, 5)
        grid.addWidget(QtWidgets.QLabel('Meas step (us)'), 3, 3)
        self.meas_step_spin = QtWidgets.QSpinBox(); self.meas_step_spin.setRange(100, 10000000); self.meas_step_spin.setSingleStep(100); self.meas_step_spin.setValue(100000)
        grid.addWidget(self.meas_step_spin, 3, 4)
        grid.addWidget(QtWidgets.QLabel('Graph width (s)'), 4, 3)
        self.graph_width_spin = QtWidgets.QSpinBox(); self.graph_width_spin.setRange(1,600); self.graph_width_spin.setValue(10)
        grid.addWidget(self.graph_width_spin, 4, 4)
        # sensor selection
        self.sens_lidar_cb = QtWidgets.QCheckBox('LIDAR'); self.sens_lidar_cb.setChecked(True)
        self.sens_imu_cb = QtWidgets.QCheckBox('IMU_ACCEL'); self.sens_imu_cb.setChecked(True)
        self.sens_cond_cb = QtWidgets.QCheckBox('CONDUCTIVITY'); self.sens_cond_cb.setChecked(False)
        grid.addWidget(self.sens_lidar_cb, 5, 3)
        grid.addWidget(self.sens_imu_cb, 5, 4)
        grid.addWidget(self.sens_cond_cb, 6, 3)

        self.up_btn = QtWidgets.QPushButton('Move Up')
        self.down_btn = QtWidgets.QPushButton('Move Down')
        self.up_btn.clicked.connect(lambda: self.send_command('BUTTON UP'))
        self.down_btn.clicked.connect(lambda: self.send_command('BUTTON DOWN'))

        self.stop_btn = QtWidgets.QPushButton('STOP')
        self.stop_btn.clicked.connect(lambda: self.send_command('STOP'))

        self.log_view = QtWidgets.QPlainTextEdit(); self.log_view.setReadOnly(True)

        # Sensor plot (matplotlib) - small canvas to the right of controls
        self.sensor_fig = Figure(figsize=(4, 3))
        self.sensor_canvas = FigureCanvas(self.sensor_fig)
        self.sensor_ax = self.sensor_fig.add_subplot(111)
        # per-sensor history: key -> deque of (ts_s, value)
        self.sensor_history: Dict[str, deque] = {}
        self._last_sensor_plot = 0.0
        self.clear_plot_btn = QtWidgets.QPushButton('Clear Plot')
        self.clear_plot_btn.clicked.connect(self.clear_plot)

        # Layout
        grid.addWidget(QtWidgets.QLabel('Serial Port'), 0, 0)
        grid.addWidget(self.port_combo, 0, 1)
        grid.addWidget(self.connect_btn, 0, 2)
        grid.addWidget(QtWidgets.QLabel('PPR'), 1, 0)
        grid.addWidget(self.ppr_combo, 1, 1)
        grid.addWidget(QtWidgets.QLabel('Wheel Dia (cm)'), 2, 0)
        grid.addWidget(self.wd_spin, 2, 1)
        grid.addWidget(QtWidgets.QLabel('Speed (cm/s)'), 3, 0)
        grid.addWidget(self.speed_spin, 3, 1)
        grid.addWidget(QtWidgets.QLabel('Duration (s)'), 4, 0)
        grid.addWidget(self.duration_spin, 4, 1)
        grid.addWidget(self.load_cfg_btn, 5, 0)
        grid.addWidget(self.send_settings_btn, 5, 1)
        grid.addWidget(self.save_cfg_btn, 5, 2)
        grid.addWidget(self.up_btn, 6, 0)
        grid.addWidget(self.down_btn, 6, 1)
        grid.addWidget(self.stop_btn, 6, 2)
        grid.addWidget(self.log_view, 7, 0, 1, 3)
        # place Clear Log button under the log view, spanning the same columns
        grid.addWidget(self.clear_log_btn, 8, 0, 1, 3)
        # place sensor plot canvas to the right
        grid.addWidget(self.sensor_canvas, 7, 3, 3, 2)
        # Clear Plot button under the plot
        grid.addWidget(self.clear_plot_btn, 10, 3, 1, 2)

        # state
        self.currently_moving = False
        self.move_start_time = None
        # experiment window reference
        self.exp_window = None

        # try load standard config on startup if present (silent)
        try:
            cfg_path = self._config_std_path()
            if os.path.exists(cfg_path):
                    self.load_std_config()
                    # attempt autoconnect and handshake shortly after loading config
                    try:
                        QtCore.QTimer.singleShot(200, self._autoconnect_and_handshake)
                    except Exception:
                        pass
        except Exception:
            pass

    def _autoconnect_and_handshake(self):
        # if a COM port is selected from loaded config, try connect and handshake
        cp = self.port_combo.currentText()
        if not cp:
            return
        if self.serial_thread is None:
            # toggle_connect will start SerialThread
            try:
                self.toggle_connect()
            except Exception:
                pass
        # send handshake after a short delay to allow thread to open port
        try:
            QtCore.QTimer.singleShot(200, self._send_handshake)
        except Exception:
            pass

    def _send_handshake(self):
        if not self.serial_thread:
            return
        self.handshake_pending = True
        self.handshake_ok = False 
        try:
            st = self.serial_thread
            if st:
                st.write_line('HELLO MOTOR!')
        except Exception:
            print('Except_connect _handshake ')
            pass
        # timeout
        try:
            self.handshake_timer = QtCore.QTimer(self)
            self.handshake_timer.setSingleShot(True)
            self.handshake_timer.timeout.connect(self._handshake_timeout)
            self.handshake_timer.start(1000)
        except Exception:
            print('Except_connect _handshake')
            pass

    def _handshake_timeout(self):
        if getattr(self, 'handshake_pending', False) and not getattr(self, 'handshake_ok', False):
            QtWidgets.QMessageBox.warning(self, 'Handshake failed', 'No valid response from MCU (expected "Brummm...")')
        self.handshake_pending = False

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(p.device)
        # also refresh sensor COM list if present
        try:
            if getattr(self, 'sensor_port_combo', None) is not None:
                self._refresh_sensor_ports()
        except Exception:
            pass

    def _refresh_sensor_ports(self):
        try:
            self.sensor_port_combo.clear()
            ports = serial.tools.list_ports.comports()
            self.sensor_port_combo.addItem('')
            for p in ports:
                self.sensor_port_combo.addItem(p.device)
        except Exception:
            pass

    def _detect_teensy(self, vid: int = 0x16C0):
        """Auto-select the first serial port matching the given VID (default PJRC 0x16C0)."""
        try:
            ports = serial.tools.list_ports.comports()
            for p in ports:
                try:
                    if p.vid is not None and int(p.vid) == int(vid):
                        # select and inform user
                        if getattr(self, 'sensor_port_combo', None) is not None:
                            # ensure list is up-to-date
                            self._refresh_sensor_ports()
                            idx = self.sensor_port_combo.findText(p.device)
                            if idx >= 0:
                                self.sensor_port_combo.setCurrentIndex(idx)
                        QtWidgets.QMessageBox.information(self, 'Detect Teensy', f'Detected Teensy on {p.device}')
                        return
                except Exception:
                    continue
            QtWidgets.QMessageBox.warning(self, 'Detect Teensy', 'No Teensy device found (VID 0x16C0)')
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Detect Teensy', f'Detection failed: {e}')

    def toggle_connect(self):
        if self.serial_thread is None:
            port = self.port_combo.currentText()
            if not port:
                QtWidgets.QMessageBox.warning(self, 'No port', 'Please select a serial port')
                return
            self.serial_thread = SerialThread(port)
            self.serial_thread.line_received.connect(self.on_line)
            self.serial_thread.start()
            self.connect_btn.setText('Disconnect')
            self.log('Connected to ' + port)
            # send current configuration to MCU after connecting
            try:
                self.send_settings(include_duration=True)
            except Exception:
                pass
            # also send sensor connection attempt (informational)
            try:
                ip = self.sensor_ip.text() if hasattr(self, 'sensor_ip') else ''
                port_s = str(self.sensor_port.value()) if hasattr(self, 'sensor_port') else ''
                if ip:
                    self.log(f'Sensor target: {ip}:{port_s}')
            except Exception:
                pass
        else:
            self.serial_thread.close()
            self.serial_thread.wait(500)
            self.serial_thread = None
            self.connect_btn.setText('Connect')
            self.log('Disconnected')

    def send_command(self, cmd: str):
        self.log('>> ' + cmd)
        if self.serial_thread:
            st = self.serial_thread
            if st:
                st.write_line(cmd)

        # on manual button pressed, prepare to collect ticks
        if cmd.startswith('BUTTON'):
            self.currently_moving = True
            self.move_start_time = datetime.now()
            self.ticks = []

    def send_settings(self, include_duration: bool = True):
        ppr = self.ppr_combo.currentText()
        wd = self.wd_spin.value()
        sp = self.speed_spin.value()
        self.send_command(f'SET PPR {ppr}')
        self.send_command(f'SET WD {wd}')
        self.send_command(f'SET SPEED {sp}')
        # optionally send configured run duration in seconds
        if include_duration:
            try:
                dur = int(self.duration_spin.value())
                self.send_command(f'SET DURATION {dur}')
            except Exception:
                pass

    # --- Menu / action handlers ---
    def open_movement_data(self):
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Open Movement data', '', 'CSV Files (*.csv);;All Files (*)')
        if not fname:
            return
        # parse CSV: try common headers, fall back to two-column numeric
        times_ms = []
        speeds = []
        try:
            with open(fname, 'r', newline='') as f:
                reader = csv.reader(f)
                header = next(reader)
                # normalize header
                h = [c.strip().lower() for c in header]
                if ('elapsed_ms' in h and ('speed' in h or 'speed_cm_s' in h or 'sum' in h)) or ('time_s' in h and ('speed' in h or 'speed_cm_s' in h or 'sum' in h)):
                    # use DictReader for robustness
                    f.seek(0)
                    dreader = csv.DictReader(f)
                    for row in dreader:
                        # determine time
                        if 'time_s' in row and row['time_s']:
                            t = float(row['time_s']) * 1000.0
                        elif 'elapsed_ms' in row and row['elapsed_ms']:
                            t = float(row['elapsed_ms'])
                        else:
                            vals = list(row.values())
                            t = float(vals[0]) * 1000.0
                        # determine speed (accept 'sum' header used by movement files)
                        if 'sum' in row and row['sum']:
                            s = float(row['sum'])
                        elif 'speed' in row and row['speed']:
                            s = float(row['speed'])
                        elif 'speed_cm_s' in row and row['speed_cm_s']:
                            s = float(row['speed_cm_s'])
                        elif 'speed_cm/s' in row:
                            s = float(row['speed_cm/s'])
                        else:
                            vals = list(row.values())
                            s = float(vals[1])
                        times_ms.append(t)
                        speeds.append(s)
                else:
                    # no header matching; treat file as two numeric columns
                    # first already read as header; try parse it as numbers
                    try:
                        a = float(header[0]); b = float(header[1])
                        times_ms.append(a); speeds.append(b)
                    except Exception:
                        pass
                    for r in reader:
                        if len(r) >= 2:
                            try:
                                t = float(r[0]); s = float(r[1])
                                # assume time is seconds if small values; convert to ms if needed
                                if t < 1.0:
                                    t = t * 1000.0
                                times_ms.append(t); speeds.append(s)
                            except Exception:
                                continue
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Open failed', f'Failed to open movement data: {e}')
            return

        if not times_ms:
            QtWidgets.QMessageBox.information(self, 'No data', 'No numeric movement data found in file')
            return

        self.last_movement_path = fname
        self.log(f'Opened movement data: {fname} ({len(times_ms)} rows)')
        self._status_bar.showMessage(f'Loaded movement data: {os.path.basename(fname)}')

        # Show dialog with plot and upload/discard options
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle('Movement Data Preview')
        dlg.resize(700, 400)
        vbox = QtWidgets.QVBoxLayout(dlg)
        fig = Figure(figsize=(5,3))
        canvas = FigureCanvas(fig)
        ax = fig.add_subplot(111)
        ax.plot(times_ms, speeds, '-o')
        ax.set_xlabel('time (ms)')
        ax.set_ylabel('speed (cm/s)')
        ax.grid(True)
        vbox.addWidget(canvas)

        btn_h = QtWidgets.QHBoxLayout()
        upload_btn = QtWidgets.QPushButton('Upload to MCU')
        discard_btn = QtWidgets.QPushButton('Discard')
        btn_h.addStretch(1)
        btn_h.addWidget(upload_btn)
        btn_h.addWidget(discard_btn)
        vbox.addLayout(btn_h)

        # upload routine: MCU-driven upload (send next row only after READY_FOR_ROW)
        def start_upload():
            if not self.serial_thread:
                QtWidgets.QMessageBox.warning(dlg, 'Not connected', 'Please connect to a serial port before uploading')
                return
            upload_btn.setEnabled(False)
            discard_btn.setEnabled(False)
            # progress bar for upload
            progress = QtWidgets.QProgressBar()
            progress.setMaximum(len(times_ms))
            progress.setValue(0)
            vbox.insertWidget(1, progress)
            # store upload state on the MainWindow so on_line/_mcu_upload_send_next can access it
            self.upload_rows = list(zip(times_ms, speeds))
            self.upload_idx = 0
            self.upload_dialog = dlg
            self.upload_progress = progress
            self.upload_in_progress = True

            # signal MCU start of data; MCU will reply with READY_FOR_ROW
            try:
                st = self.serial_thread
                if st:
                    st.write_line('BEGIN_MOVEDATA')
            except Exception:
                pass

        upload_btn.clicked.connect(start_upload)
        discard_btn.clicked.connect(dlg.reject)

        dlg.exec_()
        # clear upload flag and remove progress when dialog closes
        try:
            self.upload_in_progress = False
        except Exception:
            pass

    def open_configuration(self):
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Open Configuration', '', 'JSON Files (*.json);;All Files (*)')
        if not fname:
            return
        try:
            with open(fname, 'r') as f:
                cfg = json.load(f)
            if 'ppr' in cfg:
                idx = self.ppr_combo.findText(str(cfg['ppr']))
                if idx >= 0:
                    self.ppr_combo.setCurrentIndex(idx)
            if 'wheel_diameter_cm' in cfg:
                try:
                    self.wd_spin.setValue(float(cfg['wheel_diameter_cm']))
                except Exception:
                    pass
            self.log(f'Loaded config: {fname}')
            self._status_bar.showMessage('Configuration loaded')
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Load failed', f'Failed to load configuration: {e}')

    def _mcu_upload_send_next(self):
        # Called when MCU signals READY_FOR_ROW or when we need to advance upload.
        if not getattr(self, 'upload_in_progress', False):
            return
        i = getattr(self, 'upload_idx', 0)
        rows = getattr(self, 'upload_rows', [])
        dlg = getattr(self, 'upload_dialog', None)
        prog = getattr(self, 'upload_progress', None)
        if i >= len(rows):
            # finished - tell MCU
            try:
                st = self.serial_thread
                if st:
                    st.write_line('END_MOVEDATA')
            except Exception:
                pass
            try:
                if dlg:
                    QtWidgets.QMessageBox.information(dlg, 'Upload', 'Upload complete')
                    dlg.accept()
            except Exception:
                pass
            # clear upload state
            self.upload_in_progress = False
            self.upload_rows = []
            self.upload_idx = 0
            self.upload_dialog = None
            self.upload_progress = None
            return

        t, s = rows[i]
        try:
            st = self.serial_thread
            if st:
                st.write_line(f'MOVE {int(t)},{float(s)}')
            if prog:
                try:
                    prog.setValue(i + 1)
                except Exception:
                    pass
        except Exception as e:
            self.log(f'Upload error: {e}')
        self.upload_idx = i + 1

    # --- Sensor control methods ---
    def toggle_sensor_connect(self):
        # try a quick TCP connect check to the sensor MCU
        ip = self.sensor_ip.text()
        port = int(self.sensor_port.value())
        # if sensor COM combo has a selection, use that for serial connect
        if getattr(self, 'sensor_port_combo', None) is not None:
            serport = self.sensor_port_combo.currentText()
            if serport:
                try:
                    s = serial.Serial(serport, 115200, timeout=0.2)
                    # send STATUS handshake and read any immediate replies
                    try:
                        s.reset_input_buffer()
                    except Exception:
                        pass
                    try:
                        s.write(b'STATUS\n')
                        s.flush()
                    except Exception:
                        pass
                    # collect replies for up to 500 ms
                    deadline = _time.time() + 0.5
                    reply = None
                    buf = b''
                    while _time.time() < deadline:
                        try:
                            part = s.readline()
                        except Exception:
                            part = b''
                        if part:
                            try:
                                line = part.decode('utf-8', errors='ignore').strip()
                            except Exception:
                                line = str(part)
                            if line:
                                # take the first non-empty line as reply
                                reply = line
                                break
                    try:
                        s.close()
                    except Exception:
                        pass
                    if reply:
                        QtWidgets.QMessageBox.information(self, 'Sensors', f'Response from {serport}: {reply}')
                        # if STATUS-style response, parse and display succinctly in status bar
                        if reply.startswith('STATUS'):
                            try:
                                parts = reply.split(':')
                                info = parts[1] if len(parts) > 1 else reply
                                self._status_bar.showMessage(f'Sensor {serport}: {info}')
                            except Exception:
                                pass
                    else:
                        QtWidgets.QMessageBox.information(self, 'Sensors', f'Connected to sensors MCU at {serport} (no immediate reply)')
                except Exception as e:
                    QtWidgets.QMessageBox.warning(self, 'Sensors', f'Failed to open serial port {serport}: {e}')
                return
        # support serial connection specifier: 'serial:COM3' or 'serial:/dev/ttyACM0' in text field
        if ip.lower().startswith('serial:'):
            serport = ip.split(':', 1)[1]
            try:
                s = serial.Serial(serport, 115200, timeout=1)
                s.close()
                QtWidgets.QMessageBox.information(self, 'Sensors', f'Connected to sensors MCU at {serport}')
            except Exception as e:
                QtWidgets.QMessageBox.warning(self, 'Sensors', f'Failed to open serial port {serport}: {e}')
            return

        try:
            s = socket.create_connection((ip, port), timeout=0.5)
            s.close()
            QtWidgets.QMessageBox.information(self, 'Sensors', f'Connected to sensors MCU at {ip}:{port}')
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Sensors', f'Failed to connect to sensors MCU: {e}')

    def toggle_sensor_sampling(self):
        """Start/stop sensor sampling thread (serial or TCP) and display values in the graph."""
        try:
            # if already running, stop it
            if getattr(self, 'sensor_thread', None) is not None and self.sensor_thread.isRunning():
                try:
                    self.sensor_thread.stop()
                except Exception:
                    pass
                try:
                    self.sensor_thread.wait(500)
                except Exception:
                    pass
                self.sensor_thread = None
                self.sensor_start_btn.setText('Start Sensors')
                self.log('Sensor sampling stopped')
                return

            # start new sensor thread using the same policy as experiments
            ip = self.sensor_ip.text()
            port = int(self.sensor_port.value())
            meas_step = int(self.meas_step_spin.value())
            sensors = {'LIDAR': self.sens_lidar_cb.isChecked(), 'IMU_ACCEL': self.sens_imu_cb.isChecked(), 'CONDUCTIVITY': self.sens_cond_cb.isChecked()}
            if getattr(self, 'sensor_port_combo', None) is not None and self.sensor_port_combo.currentText():
                serport = self.sensor_port_combo.currentText()
                self.sensor_thread = SerialSensorThread(serport, 115200, meas_step, sensors)
            elif ip.lower().startswith('serial:'):
                serport = ip.split(':', 1)[1]
                self.sensor_thread = SerialSensorThread(serport, 115200, meas_step, sensors)
            else:
                self.sensor_thread = SensorThread(ip, port, meas_step, sensors)

            self.sensor_thread.measurement.connect(self._on_sensor_measurement)
            self.sensor_thread.start()
            self.sensor_start_btn.setText('Stop Sensors')
            self.log('Sensor sampling started')
        except Exception as e:
            self.log(f'Failed to toggle sensor sampling: {e}')

    def _start_sensors_for_experiment(self):
        # start sensor thread with current settings
        try:
            if getattr(self, 'sensor_thread', None) and self.sensor_thread.isRunning():
                return
            ip = self.sensor_ip.text()
            port = int(self.sensor_port.value())
            meas_step = int(self.meas_step_spin.value())
            sensors = {'LIDAR': self.sens_lidar_cb.isChecked(), 'IMU_ACCEL': self.sens_imu_cb.isChecked(), 'CONDUCTIVITY': self.sens_cond_cb.isChecked()}
            # if sensor_ip starts with 'serial:' use SerialSensorThread
            # prefer sensor COM combo if a port was selected
            if getattr(self, 'sensor_port_combo', None) is not None and self.sensor_port_combo.currentText():
                serport = self.sensor_port_combo.currentText()
                self.sensor_thread = SerialSensorThread(serport, 115200, meas_step, sensors)
            elif ip.lower().startswith('serial:'):
                serport = ip.split(':', 1)[1]
                self.sensor_thread = SerialSensorThread(serport, 115200, meas_step, sensors)
            else:
                self.sensor_thread = SensorThread(ip, port, meas_step, sensors)
            self.sensor_thread.measurement.connect(self._on_sensor_measurement)
            self.sensor_thread.start()
            self.log('Sensor sampling started')
        except Exception as e:
            self.log(f'Failed to start sensors: {e}')

    def _stop_sensors_for_experiment(self):
        try:
            if getattr(self, 'sensor_thread', None):
                try:
                    self.sensor_thread.stop()
                except Exception:
                    pass
                try:
                    self.sensor_thread.wait(500)
                except Exception:
                    pass
                self.sensor_thread = None
            self.log('Sensor sampling stopped')
        except Exception as e:
            self.log(f'Failed to stop sensors: {e}')

    def _on_sensor_measurement(self, meas: dict):
        # Update sensor history and redraw plot at most every 500 ms.
        try:
            ts_us = int(meas.get('ts_us', 0))
            ts_s = float(ts_us) / 1e6 if ts_us else _time.time()
            for k, v in list(meas.items()):
                if k == 'ts_us':
                    continue
                try:
                    # only store numeric values
                    val = float(v) if v is not None and v != '' else None
                except Exception:
                    val = None
                if val is None:
                    continue
                if k not in self.sensor_history:
                    self.sensor_history[k] = deque(maxlen=500)
                self.sensor_history[k].append((ts_s, val))

            now = _time.time()
            if now - getattr(self, '_last_sensor_plot', 0.0) >= 0.5:
                self._last_sensor_plot = now
                try:
                    self._redraw_sensor_plot()
                except Exception:
                    pass
        except Exception:
            pass

    def clear_plot(self):
        try:
            self.sensor_history.clear()
            try:
                self._redraw_sensor_plot()
            except Exception:
                pass
        except Exception:
            pass

    def _redraw_sensor_plot(self):
        # clear and plot each sensor history
        ax = self.sensor_ax
        ax.clear()
        if not self.sensor_history:
            # Differentiate between "not started" and "running but no data yet"
            try:
                if getattr(self, 'sensor_thread', None) is not None and getattr(self.sensor_thread, 'isRunning', lambda: False)():
                    ax.set_title('Waiting for sensor data...')
                else:
                    ax.set_title('No sensor data - press "Start Sensors"')
            except Exception:
                ax.set_title('No sensor data')
            ax.grid(True)
            self.sensor_canvas.draw_idle()
            return
        latest_t = 0.0
        for dq in self.sensor_history.values():
            if dq:
                latest_t = max(latest_t, dq[-1][0])
        # determine time window (seconds) and plot each sensor as time (s, relative) vs value
        try:
            width_s = float(self.graph_width_spin.value())
        except Exception:
            width_s = 10.0
        start_t = latest_t - width_s
        for k, dq in self.sensor_history.items():
            if not dq:
                continue
            # filter to window
            pts = [(t, v) for (t, v) in dq if t >= start_t]
            if not pts:
                continue
            xs = [(t - latest_t) for (t, _) in pts]
            ys = [v for (_, v) in pts]
            ax.plot(xs, ys, label=k)
        # show latest numeric values as a small textbox in the top-left
        try:
            lines = []
            # prefer grouped display: LIDAR, IMU, CONDUCTIVITY
            if 'LIDAR' in self.sensor_history and self.sensor_history['LIDAR']:
                lines.append(f"LIDAR={self.sensor_history['LIDAR'][-1][1]:.2f} cm")
            imu_keys = ['IMU_AX', 'IMU_AY', 'IMU_AZ']
            if any(k in self.sensor_history and self.sensor_history[k] for k in imu_keys):
                ax_im = []
                for k in imu_keys:
                    if k in self.sensor_history and self.sensor_history[k]:
                        ax_im.append(f"{k.split('_')[-1]}={self.sensor_history[k][-1][1]:.3f}")
                if ax_im:
                    lines.append('IMU: ' + ', '.join(ax_im))
            if 'CONDUCTIVITY' in self.sensor_history and self.sensor_history['CONDUCTIVITY']:
                lines.append(f"COND={self.sensor_history['CONDUCTIVITY'][-1][1]:.3f}")
            if lines:
                txt = '\n'.join(lines)
                ax.text(0.02, 0.98, txt, transform=ax.transAxes, va='top', ha='left', fontsize='small', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
        except Exception:
            pass
        # set x-axis to show [ -width_s .. 0 ] (relative seconds)
        try:
            ax.set_xlim(-width_s, 0)
        except Exception:
            pass
        ax.legend(loc='upper right', fontsize='small')
        ax.grid(True)
        ax.set_xlabel('time (s, relative)')
        # autoscale Y to shown data
        try:
            ax.relim()
            ax.autoscale_view()
        except Exception:
            pass
        self.sensor_canvas.draw_idle()

    def save_experiment_results(self):
        if not self.ticks:
            QtWidgets.QMessageBox.information(self, 'No data', 'No experiment data to save')
            return
        fname, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Save Experiment results', '', 'CSV Files (*.csv);;All Files (*)')
        if not fname:
            return
        try:
            with open(fname, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['elapsed_ms', 'speed_cm_s'])
                for r in self.ticks:
                    writer.writerow([r[0], r[1]])
            self.log(f'Saved experiment results: {fname}')
            self._status_bar.showMessage(f'Saved {os.path.basename(fname)}')
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Save failed', f'Failed to save results: {e}')

    def load_calibration(self):
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(self, 'Load Calibration', '', 'JSON Files (*.json);;All Files (*)')
        if not fname:
            return
        try:
            with open(fname, 'r') as f:
                cal = json.load(f)
            if 'ppr' in cal:
                idx = self.ppr_combo.findText(str(cal['ppr']))
                if idx >= 0:
                    self.ppr_combo.setCurrentIndex(idx)
            if 'wheel_diameter_cm' in cal:
                try:
                    self.wd_spin.setValue(float(cal['wheel_diameter_cm']))
                except Exception:
                    pass
            self.log(f'Calibration loaded: {fname}')
            self._status_bar.showMessage('Calibration loaded')
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Load failed', f'Failed to load calibration: {e}')

    def start_calibration(self):
        # Open calibration dialog
        try:
            self.calib_window = CalibrationWindow(self)
            self.calib_window.show()
            self.log('Calibration dialog opened')
            self._status_bar.showMessage('Calibration open')
        except Exception:
            # fallback behaviour
            self.log('Failed to open calibration dialog')
            try:
                self.send_command('START_CALIBRATION')
                self._status_bar.showMessage('Calibration started')
            except Exception:
                pass

    def motor_data(self):
        # Show a simple dialog with current motor settings
        ppr = self.ppr_combo.currentText()
        wd = self.wd_spin.value()
        QtWidgets.QMessageBox.information(self, 'Motor data', f'PPR: {ppr}\nWheel diameter (cm): {wd}')

    def start_experiment(self):
        # Open the experiment control window (user starts/stops there)
        try:
            self.exp_window = ExperimentWindow(self)
            self.exp_window.show()
            # if sensors should collect on experiment start, wire signal
            try:
                # connect experiment start/stop to sensor collection
                self.exp_window.start_btn.clicked.connect(lambda: self._start_sensors_for_experiment())
                self.exp_window.stop_btn.clicked.connect(lambda: self._stop_sensors_for_experiment())
            except Exception:
                pass
        except Exception:
            # fallback behaviour: send settings and start
            self.send_settings()
            self.send_command('START_EXPERIMENT')
            self.log('Experiment started')
            self._status_bar.showMessage('Experiment running')

    def _config_std_path(self):
        # program home dir = parent of this script directory
        base = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        return os.path.join(base, 'config_std.json')

    def save_std_config(self):
        path = self._config_std_path()
        cfg = {
            'com_port': self.port_combo.currentText(),
            'ppr': int(self.ppr_combo.currentText()) if self.ppr_combo.currentText().isdigit() else None,
            'wheel_diameter_cm': float(self.wd_spin.value()),
            'speed_cm_s': float(self.speed_spin.value()),
            'duration_s': int(self.duration_spin.value())
        }
        try:
            with open(path, 'w') as f:
                json.dump(cfg, f, indent=2)
            self.log(f'Saved config to {path}')
            self._status_bar.showMessage(f'Saved config: {os.path.basename(path)}')
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Save failed', f'Failed to save config: {e}')

    def load_std_config(self):
        path = self._config_std_path()
        if not os.path.exists(path):
            QtWidgets.QMessageBox.information(self, 'No config', f'No config file found at {path}')
            return
        try:
            with open(path, 'r') as f:
                cfg = json.load(f)
            # apply COM port (add to combo if missing)
            cp = cfg.get('com_port')
            if cp:
                idx = self.port_combo.findText(cp)
                if idx == -1:
                    self.port_combo.addItem(cp)
                    idx = self.port_combo.findText(cp)
                if idx >= 0:
                    self.port_combo.setCurrentIndex(idx)
            # apply PPR
            ppr = cfg.get('ppr')
            if ppr is not None:
                idx = self.ppr_combo.findText(str(ppr))
                if idx >= 0:
                    self.ppr_combo.setCurrentIndex(idx)
            # wheel diameter, speed, duration
            if 'wheel_diameter_cm' in cfg:
                try:
                    self.wd_spin.setValue(float(cfg.get('wheel_diameter_cm')))
                except Exception:
                    pass
            if 'speed_cm_s' in cfg:
                try:
                    self.speed_spin.setValue(float(cfg.get('speed_cm_s')))
                except Exception:
                    pass
            if 'duration_s' in cfg:
                try:
                    self.duration_spin.setValue(int(cfg.get('duration_s')))
                except Exception:
                    pass
            self.log(f'Loaded config from {path}')
            self._status_bar.showMessage(f'Loaded config: {os.path.basename(path)}')
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, 'Load failed', f'Failed to load config: {e}')

    def show_help(self):
        QtWidgets.QMessageBox.information(self, 'Help', 'See README and documentation for usage details.')

    def show_about(self):
        QtWidgets.QMessageBox.information(self, 'About', 'Aufzug Main Test\nAuthor: A. B\nVersion: 0.01')
    def on_line(self, line: str):
        # if a handshake is pending, check for expected response or error
        if getattr(self, 'handshake_pending', False):
            # exact expected reply
            if line.strip() == 'Brummm...':
                self.handshake_ok = True
                self.handshake_pending = False
                try:
                    self.handshake_timer.stop()
                except Exception:
                    pass
                QtWidgets.QMessageBox.information(self, 'MCU connected', f'Connected to MCU on {self.port_combo.currentText()}')
                return
            # immediate error from serial thread
            if line.startswith('ERROR:SERIAL_OPEN') or line.startswith('ERROR:SERIAL_READ'):
                self.handshake_pending = False
                try:
                    self.handshake_timer.stop()
                except Exception:
                    pass
                QtWidgets.QMessageBox.warning(self, 'Handshake failed', f'Error during connect: {line}')
                return
        # suppress upload chatter: don't display per-row upload logs or READY_FOR_ROW when uploading
        if getattr(self, 'upload_in_progress', False):
            # MCU signals readiness for the next row — handle silently
            if line.startswith('READY_FOR_ROW'):
                try:
                    self._mcu_upload_send_next()
                except Exception:
                    pass
                return
            # suppress per-row ACKs from MCU or echoed upload notifications from GUI
            if line.startswith('ACK:MOVEDATA:ROW') or line.startswith('ACK:MOVEDATA:BEGIN') or line.startswith('ACK:MOVEDATA:END'):
                return
        # sensor forwarding: SENSOR:HEIGHT_CM=<value>
        try:
            if line.upper().startswith('SENSOR:HEIGHT_CM'):
                # accept formats SENSOR:HEIGHT_CM=123.4 or SENSOR:HEIGHT_CM:123.4
                sep = '=' if '=' in line else ':'
                parts = line.split(sep, 1)
                if len(parts) >= 2:
                    try:
                        val = float(parts[1])
                        # prefer typed attribute access for static analysis
                        if self.calib_window is not None:
                            try:
                                self.calib_window.update_sensor_height(val)
                            except Exception:
                                pass
                        # still log the sensor line
                    except Exception:
                        pass
        except Exception:
            pass
        # suppress verbose experiment update lines from appearing in the log window
        if not line.startswith('DBG:EXPERIMENT:UPDATE'):
            self.log('< ' + line)
        # handle VTABLE ACK responses: ACK:VTABLE:LEN=<n>
        try:
            if line.startswith('ACK:VTABLE:') and self.exp_window is not None:
                # parse LEN if present
                parts = line.split(':', 2)
                if len(parts) >= 3:
                    payload = parts[2]
                    # payload like LEN=123 or 0
                    length = 0
                    if payload.upper().startswith('LEN='):
                        try:
                            length = int(payload.split('=',1)[1])
                        except Exception:
                            length = 0
                    else:
                        try:
                            length = int(payload)
                        except Exception:
                            length = 0
                    try:
                        if self.exp_window is not None:
                            self.exp_window.update_vtable_status(length)
                    except Exception:
                        pass
                    return
        except Exception:
            pass
        if line.startswith('TICK'):
            # format TICK:elapsed_ms,speed
            print('Tick received:', line)
            try:
                print('Parsing tick line')
                payload = line[4:]
                print('Payload: ', payload)
                elapsed_ms_str, speed_str = payload.split(',')
                print('Elapsed_STR: ', elapsed_ms_str)
                elapsed_ms = int(elapsed_ms_str)
                speed = float(speed_str)
                self.ticks.append((elapsed_ms, speed))
                # forward tick to experiment window if present
                if self.exp_window is not None:
                    try:
                        self.exp_window.append_tick(elapsed_ms, speed)
                    except Exception:
                        pass
            except Exception:
                print('Failed to parse TICK line:', line)
                pass
        elif line.startswith('DONE:MOVE') or line.startswith('ERROR:'):
            # save CSV if we were collecting           
            if self.currently_moving:
                self.currently_moving = False
                now = datetime.now()
                fname = now.strftime('motor_run_%Y%m%d_%H%M%S.csv')
                try:
                    with open(fname, 'w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(['elapsed_ms', 'speed_cm_s'])
                        for r in self.ticks:
                            writer.writerow([r[0], r[1]])
                    full = os.path.abspath(fname)
                    self.log(f'Saved CSV {full} ({len(self.ticks)} rows)')
                except Exception as e:
                    self.log(f'Failed to save CSV: {e}')
        
        # handle experiment DBG and DONE messages
        if line.startswith('DBG:'):
            # experiment-specific DBG handling
            try:
                if line.startswith('DBG:EXPERIMENT:UPDATE'):
                    parts = line.split(':', 3)
                    if len(parts) >= 4:
                        payload = parts[3]
                        kvs = {}
                        for p in payload.split(','):
                            if '=' in p:
                                k, v = p.split('=', 1)
                                kvs[k.strip().upper()] = v.strip()
                        if self.exp_window is not None:
                            try:
                                self.exp_window.append_dbg_update(kvs)
                            except Exception:
                                pass
                elif line.startswith('DBG:EXPERIMENT:RESTART'):
                    parts = line.split(':', 3)
                    if len(parts) >= 4:
                        payload = parts[3]
                        for p in payload.split(','):
                            if '=' in p:
                                k, v = p.split('=', 1)
                                if k.strip().upper() == 'LEFT' and self.exp_window is not None:
                                    try:
                                        self.exp_window.append_restart(int(v.strip()))
                                    except Exception:
                                        pass
                elif line.startswith('DBG:EXPERIMENT:INFO'):
                    # MCU provides experiment info: RUNDUR_MS=...,REPS_TOTAL=...,REPS_LEFT=...,VT_LEN=...
                    parts = line.split(':', 3)
                    if len(parts) >= 4:
                        payload = parts[3]
                        kvs = {}
                        for p in payload.split(','):
                            if '=' in p:
                                k, v = p.split('=', 1)
                                kvs[k.strip().upper()] = v.strip()
                        if self.exp_window is not None:
                            try:
                                rt = kvs.get('REPS_TOTAL')
                                rl = kvs.get('REPS_LEFT')
                                if rt is not None and rt != '':
                                    try:
                                        self.exp_window.reps_total = int(rt)
                                    except Exception:
                                        pass
                                if rl is not None and rl != '' and getattr(self.exp_window, 'reps_total', None):
                                    try:
                                        left_i = int(rl)
                                        run_idx = (self.exp_window.reps_total - left_i) + 1
                                        if run_idx < 1:
                                            run_idx = 1
                                        if run_idx > self.exp_window.reps_total:
                                            run_idx = self.exp_window.reps_total
                                        try:
                                            self.exp_window.lbl_status.setText(f'Running ({run_idx}/{self.exp_window.reps_total})')
                                        except Exception:
                                            pass
                                    except Exception:
                                        pass
                            except Exception:
                                pass
                elif line.startswith('DBG:EXPERIMENT:START'):
                    if self.exp_window is not None:
                        try:
                            self.exp_window.lbl_status.setText(f'Running (1/{self.exp_window.reps_total})')
                            self.exp_window.running = True
                        except Exception:
                            pass
            except Exception:
                pass
            return

        # DONE:EXPERIMENT - allow experiment window to handle repetitions
        if line.startswith('DONE:EXPERIMENT'):
            try:
                reason = None
                parts = line.split(':', 2)
                if len(parts) >= 3:
                    payload = parts[2]
                    for p in payload.split(','):
                        if '=' in p:
                            k, v = p.split('=', 1)
                            if k.strip().upper() == 'REASON':
                                reason = v.strip()
                if self.exp_window is not None:
                    try:
                        self.exp_window.handle_done_experiment(reason)
                    except Exception:
                        pass
                # stop sensor acquisition when experiment finishes
                try:
                    self._stop_sensors_for_experiment()
                except Exception:
                    pass
            except Exception:
                pass
    def log(self, text: str):
        ts = datetime.now().strftime('%H:%M:%S.%f')
        self.log_view.appendPlainText(f'[{ts}] {text}')

    def clear_log(self):
        try:
            self.log_view.clear()
            self._status_bar.showMessage('Log cleared')
        except Exception:
            pass

    def closeEvent(self, a0):
        if self.serial_thread:
            self.serial_thread.close()
            self.serial_thread.wait(500)
        super().closeEvent(a0)


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
