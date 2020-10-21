from PyQt5 import QtCore, QtWidgets
from gui import Ui_MainWindow
import sys
from sensorFusion import SensorFusion
import numpy as np
from PyQt5.QtWidgets import QMessageBox
from serial import SerialException, SerialTimeoutException
import uart_config_dialog

SAMPLE_PERIOD = 0.01
FRAME_RATE = 60


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.ui = Ui_MainWindow(self)

        self.ui.actionConnect.triggered.connect(self.uart_connect)
        self.ui.actionRun.triggered.connect(self.plot_run)
        self.ui.actionInfo.triggered.connect(self.help_msg)

        self.sensor_fusion = SensorFusion()

        self.update_plot()

        self.show()

        # Setup a timer to trigger the redraw by calling update_plot.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(round(1000/FRAME_RATE))
        self.timer.timeout.connect(self.update_plot)

        self.show()

    def update_plot(self):
        # Drop off the first y element, append a new one.
        try:
            data = self.sensor_fusion.get_data()
        except (SerialException, SerialTimeoutException):
            self.uart_error()
            self.timer.stop()
            return

        if len(data['acc']) > 0:
            roll, pitch = self.sensor_fusion.get_angles(data)

            it = zip(
                (
                    roll, pitch, *(data['acc'].swapaxes(0, 1)), *(data['gyro'].swapaxes(0, 1))
                ),
                (
                    self.ui.graphicsView_roll_t, self.ui.graphicsView_pitch_t,
                    self.ui.graphicsView_acc_x_t, self.ui.graphicsView_acc_y_t, self.ui.graphicsView_acc_z_t,
                    self.ui.graphicsView_gyro_x_t, self.ui.graphicsView_gyro_y_t, self.ui.graphicsView_gyro_z_t
                ),
                (
                    self.ui.graphicsView_roll_f, self.ui.graphicsView_pitch_f,
                    self.ui.graphicsView_acc_x_f, self.ui.graphicsView_acc_y_f, self.ui.graphicsView_acc_z_f,
                    self.ui.graphicsView_gyro_x_f, self.ui.graphicsView_gyro_y_f, self.ui.graphicsView_gyro_z_f
                )
            )

            for ydata, time_plot, freq_plot in it:
                time_plot.update_plot(ydata)

                fft = np.fft.fftshift(np.abs(np.fft.fft(time_plot.ydata))**2)
                f = np.fft.fftshift(np.fft.fftfreq(len(time_plot.ydata), d=0.01))
                fft = fft[f > 0]
                freq_plot.update_plot(fft/np.max(fft))

    def plot_run(self):
        if self.ui.actionRun.isChecked():
            if self.ui.actionConnect.isChecked():
                self.sensor_fusion.start()
                self.timer.start()
            else:
                self.ui.actionRun.setChecked(False)
                self.unconnected_error()
        else:
            self.plot_stop()

    def unconnected_error(self):
        msg = QMessageBox()
        msg.setWindowTitle('Error')
        msg.setText('Please connect to a serial port')
        msg.setIcon(QMessageBox.Warning)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.setDetailedText("To connect to a serial port, go to Settings -> Connect. " +
                            "If you don't see the port you need listed, check that no other apps are using it.")
        msg.exec_()

    def plot_stop(self):
        self.timer.stop()
        self.sensor_fusion.stop()
        self.ui.actionRun.setChecked(False)

    def uart_connect(self):
        if self.ui.actionConnect.isChecked():
            port = self.uart_config()
            if port != '':
                self.sensor_fusion.config(port=port)
                self.sensor_fusion.connect()
            else:
                self.ui.actionConnect.setChecked(False)
        else:
            self.uart_disconnect()

    def uart_disconnect(self):
        if self.sensor_fusion.is_connected():
            self.plot_stop()
            self.sensor_fusion.disconnect()
            self.ui.actionConnect.setChecked(False)

    def uart_error(self):
        msg = QMessageBox()
        msg.setWindowTitle('Error')
        msg.setText('Serial connection unavailable')
        msg.setIcon(QMessageBox.Critical)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()
        self.uart_disconnect()

    def uart_config(self):
        port = ''
        dialog = QtWidgets.QDialog()
        try:
            ui = uart_config_dialog.Ui_Dialog(dialog)
            if dialog.exec_():
                port = ui.selected_port

        except (SerialException, SerialTimeoutException):
            self.uart_error()

        return port

    def help_msg(self):
        msg = QMessageBox()
        msg.setWindowTitle('Info')
        msg.setText('1. Connect to a serial port (Settings -> Connect)')
        msg.setInformativeText("2. Run (Plot -> Run)")
        msg.setDetailedText(
            "The serial ports listed are detected automatically from all available ports. " +
            "If you don't see the port you need listed, make sure no other apps are using it."
        )
        msg.setIcon(QMessageBox.Information)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    sys.exit(app.exec_())
