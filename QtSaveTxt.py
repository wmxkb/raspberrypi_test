import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QFileDialog
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot
import numpy as np

class App(QWidget):

    def __init__(self):
        super().__init__()
        self.title = 'PyQt5 button - pythonspot.com'
        self.left = 10
        self.top = 10
        self.width = 320
        self.height = 200
        self.initUI()
        self.x = np.arange(0, 10, 0.01)
        self.sindata = np.sin(self.x)

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        button = QPushButton('Save data', self)
        button.setToolTip('This is an example button')
        button.move(100, 70)
        button.clicked.connect(self.on_click)

        self.show()

    @pyqtSlot()
    def on_click(self):
        filename = QFileDialog.getSaveFileName(self, 'Save Data', '*.txt')
        # name = filename.rsplit('/', 1)[-1]
        name = filename[0].rsplit('/')[-1]
        np.savetxt(name + '_Y.txt', self.sindata)
        np.savetxt(name + '_X.txt', self.x)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())