import sys
import serial
import serial.tools.list_ports
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# --- Configuration ---
BAUD_RATE = 115200
SETTLING_BAND = 0.02  # 2% Band
BUFFER_SIZE = 1000    # Number of points to show on graph

class MotorMonitor(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # UI Setup
        self.setWindowTitle("STM32 Motor Performance Monitor (Python)")
        self.resize(1000, 600)

        # Central Widget
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        # Stats Labels
        self.stats_layout = QtWidgets.QHBoxLayout()
        self.label_os = QtWidgets.QLabel("Overshoot: -- %")
        self.label_st = QtWidgets.QLabel("Settling Time: -- samples")
        self.label_sp = QtWidgets.QLabel("Setpoint: -- deg")
        self.label_pos = QtWidgets.QLabel("Position: -- deg")
        
        for lbl in [self.label_os, self.label_st, self.label_sp, self.label_pos]:
            lbl.setStyleSheet("font-size: 14px; font-weight: bold; color: #333;")
            self.stats_layout.addWidget(lbl)
        
        self.layout.addLayout(self.stats_layout)

        # Plot Setup
        self.plot_widget = pg.PlotWidget(title="Live Motor Response")
        self.plot_widget.setBackground('w')
        self.plot_widget.addLegend()
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel('left', 'Position', units='deg')
        self.plot_widget.setLabel('bottom', 'Samples')

        self.curve_actual = self.plot_widget.plot(pen=pg.mkPen('b', width=2), name="Actual Position")
        self.curve_target = self.plot_widget.plot(pen=pg.mkPen('r', width=1.5, style=QtCore.Qt.DashLine), name="Setpoint")
        
        self.layout.addWidget(self.plot_widget)

        # Data Buffers
        self.data_actual = np.zeros(BUFFER_SIZE)
        self.data_target = np.zeros(BUFFER_SIZE)
        self.ptr = 0

        # Serial Setup
        self.serial_port = self.find_stm32_port()
        if self.serial_port:
            try:
                self.ser = serial.Serial(self.serial_port, BAUD_RATE, timeout=0.1)
                print(f"Connected to {self.serial_port}")
            except Exception as e:
                print(f"Error opening serial: {e}")
                sys.exit(1)
        else:
            print("No STM32 found. Please check connection.")
            sys.exit(1)

        # Timer for Updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10) # 100Hz Update rate

    def find_stm32_port(self):
        ports = serial.tools.list_ports.comports()
        for p in ports:
            # Look for STMicroelectronics or general serial ports
            if "STMicroelectronics" in p.description or "USB Serial" in p.description:
                return p.device
        return ports[0].device if ports else None

    def calculate_metrics(self):
        if self.ptr < 50: return

        actual = self.data_actual[:self.ptr]
        target = self.data_target[:self.ptr]
        
        current_target = target[-1]
        if abs(current_target) < 0.1: return

        # 1. Overshoot
        peak = np.max(actual) if current_target > 0 else np.min(actual)
        overshoot = ((peak - current_target) / current_target) * 100
        overshoot = max(0, overshoot) if current_target > 0 else max(0, -overshoot)

        # 2. Settling Time (Last time it exited the 2% band)
        upper_bound = current_target * (1 + SETTLING_BAND)
        lower_bound = current_target * (1 - SETTLING_BAND)
        
        # Search backwards
        settled_idx = self.ptr - 1
        for i in range(self.ptr - 1, 0, -1):
            if actual[i] > max(upper_bound, lower_bound) or actual[i] < min(upper_bound, lower_bound):
                settled_idx = i
                break
        
        settling_samples = self.ptr - settled_idx

        # Update Labels
        self.label_os.setText(f"Overshoot: {overshoot:.2f} %")
        self.label_st.setText(f"Settled: {settling_samples} samples")
        self.label_sp.setText(f"Setpoint: {current_target:.1f} deg")
        self.label_pos.setText(f"Position: {actual[-1]:.1f} deg")

    def update(self):
        if not self.ser.in_waiting: return

        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line: return

            parts = line.split(',')
            if "PREVIEW" in line or "DATA" in line:
                # Actual is second to last, Target is last
                act = float(parts[-2]) / 100.0
                tar = float(parts[-1]) / 100.0

                if self.ptr < BUFFER_SIZE:
                    self.data_actual[self.ptr] = act
                    self.data_target[self.ptr] = tar
                    self.ptr += 1
                else:
                    self.data_actual[:-1] = self.data_actual[1:]
                    self.data_target[:-1] = self.data_target[1:]
                    self.data_actual[-1] = act
                    self.data_target[-1] = tar
                
                self.curve_actual.setData(self.data_actual[:self.ptr])
                self.curve_target.setData(self.data_target[:self.ptr])
                
                self.calculate_metrics()

        except Exception as e:
            pass # Handle parsing errors silently

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MotorMonitor()
    window.show()
    sys.exit(app.exec_())
