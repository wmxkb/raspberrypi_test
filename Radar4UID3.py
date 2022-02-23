from pyqtgraph.Qt import QtGui, QtCore
from pymoduleconnector import ModuleConnector
from pymoduleconnector.extras.auto import auto
from pymoduleconnector.extras.x4_regmap_autogen import X4
import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import find_peaks_cwt
import pyqtgraph as pg
import sys
import os
import glob
from time import sleep
from Guid3 import Ui_Form
import RPi.GPIO as GPIO
from detect_peaks import detect_peaks


class Form(QtGui.QWidget, Ui_Form):
    def __init__(self, parent=None):
        super(Form, self).__init__(parent)
        self.setupUi(self)
        self.setFixedSize(870, 530)
        self.setWindowTitle('Monitor')
        #############################################
        self.plt = self.pltWidget.plot()
        self.plt1 = self.pltWidget_2.plot()
        #############################################
        self.btn_connect.clicked.connect(self.X4_connect)
        self.btn_set.clicked.connect(self.X4_config_changed)
        self.btn_pause.clicked.connect(self.X4_pause)
        self.btn_resume.clicked.connect(self.X4_resume)
        self.btn_save.clicked.connect(self.savedata)
        #############################################
        self.btn_Background.clicked.connect(self.backsub)
        #############################################
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        self.Pin_out = [36, 38, 40]
        GPIO.setup(self.Pin_out, GPIO.OUT)
        #############################################
        self.rotateStep = int(float(self.ldt_resolution.text())/0.04)
        self.resolution = float(self.ldt_resolution.text())
        #############################################
        self.interpFun = None
        self.distanceO = 0
        self.distanceS = 0
        self.distanceArray = np.zeros((20,))
        self.distanceArray1 = np.zeros((20,))
        #############################################
        self.angCnt = 0
        self.angCntEnd = 0
        self.direction = 'c'
        self.r = None
        self.mc = None
        self.regmap = None
        if self.rb_base.isChecked():
            self.baseband = True
        else:
            self.baseband = False
        self.FPS = float(self.ldt_FPS.text())
        self.PRF = int(243/int(self.ldt_PRF.text()))
        # set Tx power: 0-OFF 1-Low 2-Medium 3-High
        self.tx_power = int(self.ldt_tx_power.text())
        self.dac_min = int(self.ldt_dac_min.text())
        self.dac_max = int(self.ldt_dac_max.text())
        self.iterations = int(self.ldt_iterations.text())
        self.pulses_per_step = int(self.ldt_pulse_per_step.text())
        self.range_min = float(self.ldt_range_min.text())
        self.range_max = float(self.ldt_range_max.text())
        self.scan_range = int(self.ldt_scan_range.text())
        self.n = None
        self.data = None
        self.data_new = None
        self.data_back = None
        self.data_sub = None
        self.x_axis = None
        self.x_axis_new = None
        self.distance_resolution = 1e-2
        self.pos = None
        self.scale = None
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.data_update)

        try:
            self.device_name = auto('x4')[0]
            self.X4_init()
            self.btn_save.setEnabled(False)
            self.btn_resume.setEnabled(False)
            self.btn_connect.setEnabled(False)
            self.lbl_connect.setText('Y')
            self.timer.start(1000/self.FPS)
        except:
            self.btn_connect.setEnabled(True)
            self.lbl_connect.setText('N')

    def savedata(self):
        filename = QtGui.QFileDialog.getSaveFileName(self, 'Save Data')
        name = filename.rsplit('/')[-1]
        
        try:
            np.savetxt(name + '_Y.txt', self.data_new)
            np.savetxt(name + '_X.txt', self.x_axis_new)
            print(name + " saved")
        except:
            pass
            
    def Rotate(self, n, d):
        if d == 'c':
            GPIO.output(self.Pin_out[1], True)
            for i in range(n):
                GPIO.output(self.Pin_out[0], True)
                sleep(0.0001)
                GPIO.output(self.Pin_out[0], False)
                sleep(0.0001)
        elif d == 'cc':
            GPIO.output(self.Pin_out[0], True)
            for i in range(n):
                GPIO.output(self.Pin_out[1], True)
                sleep(0.0001)
                GPIO.output(self.Pin_out[1], False)
                sleep(0.0001)

    def backsub(self):
        self.data_back = self.data_new

    def X4_pause(self):
        self.timer.stop()
        self.r.x4driver_set_fps(0)
        self.btn_save.setEnabled(True)
        self.btn_resume.setEnabled(True)
        self.btn_pause.setEnabled(False)
        self.btn_set.setEnabled(False)

    def X4_resume(self):
        self.r.x4driver_set_fps(self.FPS)
        self.timer.start(1000/self.FPS)
        self.btn_save.setEnabled(False)
        self.btn_resume.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.btn_set.setEnabled(True)

    def X4_config_changed(self):
        self.timer.stop()
        self.r.x4driver_set_fps(0)
        if self.rb_base.isChecked():
            self.baseband = True
        else:
            self.baseband = False
        self.range_min = float(self.ldt_range_min.text())
        self.range_max = float(self.ldt_range_max.text())
        self.r.x4driver_set_frame_area(self.range_min, self.range_max)
        if self.baseband:
            self.r.x4driver_set_downconversion(1)
        else:
            self.r.x4driver_set_downconversion(0)
        self.FPS = float(self.ldt_FPS.text())
        self.tx_power = int(self.ldt_tx_power.text())
        self.dac_min = int(self.ldt_dac_min.text())
        self.dac_max = int(self.ldt_dac_max.text())
        self.iterations = int(self.ldt_iterations.text())
        self.pulses_per_step = int(self.ldt_pulse_per_step.text())
        self.PRF = int(243/int(self.ldt_PRF.text()))
        self.r.x4driver_set_dac_min(self.dac_min)
        self.r.x4driver_set_dac_max(self.dac_max)
        self.r.x4driver_set_iterations(self.iterations)
        self.r.x4driver_set_pulses_per_step(self.pulses_per_step)
        self.r.x4driver_set_prf_div(self.PRF)
        self.regmap.tx_power = self.tx_power
        self.r.x4driver_set_fps(self.FPS)
        self.clear_buffer()
        self.read_frame()
        self.clear_buffer()
        self.rotateStep = int(float(self.ldt_resolution.text())/0.04)
        self.resolution = float(self.ldt_resolution.text())
        self.scan_range = int(self.ldt_scan_range.text())
        self.data = np.zeros(shape=(1, self.n))
        #################################################################
        if self.scan_range == 0:
            GPIO.output(self.Pin_out[2], False)
        elif self.scan_range == 360:
            GPIO.output(self.Pin_out[2], True)
        else:
            GPIO.output(self.Pin_out[2], True)
        #################################################################
        self.x_axis = np.linspace(self.range_min, self.range_max, self.n)
        self.x_axis_new = np.arange(self.range_min, self.range_max, self.distance_resolution)
        self.data_new = np.zeros(shape=(1, self.x_axis_new.size))
        self.data_back = np.zeros(shape=(1, self.x_axis_new.size))
        self.data_sub = np.zeros(shape=(1, self.x_axis_new.size))
        self.pos = (0, self.range_min)
        self.scale = (self.resolution, (self.range_max-self.range_min)/self.n)
        self.angCnt = 0
        self.angCntEnd = int(self.scan_range/self.resolution)
        self.timer.start(1000/self.FPS)


    def X4_connect(self):
        try:
            self.device_name = auto('x4')[0]
            self.X4_init()
            self.btn_connect.setEnabled(False)
            self.lbl_connect.setText('Y')
        except:
            self.btn_connect.setEnabled(True)
            self.lbl_connect.setText('N')


    def reset(self):
        mc = ModuleConnector(self.device_name)
        r = mc.get_xep()
        r.module_reset()
        mc.close()
        del r
        del mc
        sleep(3)

    def X4_init(self):
        self.reset()
        self.mc = ModuleConnector(self.device_name)
        self.r = self.mc.get_xep()
        self.regmap = X4(self.r)
        # set dac range
        self.r.x4driver_set_dac_min(self.dac_min)
        self.r.x4driver_set_dac_max(self.dac_max)
        # set integration
        self.r.x4driver_set_iterations(self.iterations)
        self.r.x4driver_set_pulses_per_step(self.pulses_per_step)
        # set frame area
        self.r.x4driver_set_frame_area(self.range_min, self.range_max)
        self.regmap.tx_power = self.tx_power
        self.r.x4driver_set_prf_div(self.PRF)
        if self.baseband:
            self.r.x4driver_set_downconversion(1)
        else:
            self.r.x4driver_set_downconversion(0)
        self.r.x4driver_set_fps(self.FPS)
        self.read_frame()
        self.clear_buffer()
        self.data = np.zeros(shape=(1, self.n))  
        self.x_axis = np.linspace(self.range_min, self.range_max, self.n)
        self.x_axis_new = np.arange(self.range_min, self.range_max, self.distance_resolution)
        self.data_new = np.zeros(shape=(1, self.x_axis_new.size))
        self.data_back = np.zeros(shape=(1, self.x_axis_new.size))
        #self.data_back = self.data_new
        self.data_sub = np.zeros(shape=(1, self.x_axis_new.size))
        self.pos = (0, self.range_min)
        self.scale = (self.resolution, (self.range_max-self.range_min)/self.n)
        if self.scan_range == 0:
            GPIO.output(self.Pin_out[2], False)
        self.angCnt = 0
        self.angCntEnd = int(self.scan_range/self.resolution)
        


    def clear_buffer(self):
        while self.r.peek_message_data_float():
            _ = self.r.read_message_data_float()

    def read_frame(self):
        d = self.r.read_message_data_float()
        frame = np.array(d.data)
        self.n = len(frame)
        if self.baseband: 
            frame = frame[:int(self.n/2)]+1j*frame[int(self.n/2):]
            self.n = int(self.n/2)
        return frame

    def data_update(self):
        self.data = self.read_frame()
        if self.baseband:
            self.data = abs(self.data)
                
        #######################################################
        self.interpFun = interp1d(self.x_axis, self.data, kind='cubic')
        if self.baseband:
            #self.data_new = np.power(self.interpFun(self.x_axis_new), 40)#*1e38
            self.data_new = self.interpFun(self.x_axis_new)
            # peak = detect_peaks(self.data_new, mph=0.001, mpd=1000)
            distanceO = int(round((np.where(self.data_new == self.data_new.max())[0][0]*self.distance_resolution+self.range_min)*1e3))
            self.distanceArray[1:] = self.distanceArray[:-1]
            self.distanceArray[0] = distanceO
            # self.distanceArray1[1:] = self.distanceArray1[:-1]
            # self.distanceArray1[0] = peak[0]
            distance = np.average(self.distanceArray)
            # distance1 = np.average(self.distanceArray1)
                # print(peak)
                # distance1 = (peak[1]-peak[0])/100
                # distance2 = (peak[2]-peak[1])/100
                # print("R1={}; R2={}; A1={}; A2={}".format(0, 0, peak[0]/100, distanceO/100))
            # print("D1={}, D2={}".format(distance1/100, distance/100))
            print("D={}".format(distance/100))
                # self.ldt_distance.setText(str(peak[2]/100))
                # self.ldt_distance_2.setText(str(distance))
#             except:
#                print("Error")
        else:
            self.data_new = self.interpFun(self.x_axis_new)

        # self.data_sub = abs(self.data_new - self.data_back)
        # x = self.data_new - self.data_back
        # x[x<0] = 0
        # self.data_sub = x
        # self.distanceO = int(round((np.where(self.data_new == self.data_new.max())[0][0]*self.distance_resolution+self.range_min)*10000))
        # self.distanceS = int(round((np.where(self.data_sub == self.data_sub.max())[0][0]*self.distance_resolution+self.range_min)*1000))
        
        # self.ldt_distance_2.setText(str(self.distanceS))
        #######################################################
        self.plt.setData(self.x_axis_new, self.data_new)
        # try:
        #     self.plt1.setData(self.x_axis_new, self.data_sub)
        # except:
        #     pass
        self.clear_buffer()


    def closeEvent(self, QCloseEvent):
        try:
            self.timer.stop()
            self.r.x4driver_set_fps(0)
            self.mc.close()
            del self.r
            del self.mc
            for f in glob.glob("*.log"):
                os.remove(f)
        except:
            pass



if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    f = Form()
    f.show()
    try:
        sys.exit(app.exec_())
    except:
        pass
