import sys
from typing import Counter
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

class MyWindow(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.text_edit = QTextEdit(self)
        self.text_edit.setEnabled(True)
        box_layout = QVBoxLayout()
        box_layout.addWidget(self.text_edit)
        self.setLayout(box_layout)
        self.myThread = Worker(parent=self)
        self.myThread.count_changed.connect(self.update_count)
        self.myThread.start()
#@pyqtSlot(int)
    def update_count(self, count):
        self.text_edit.append(str(count))

class Worker(QThread):
    count_changed = pyqtSignal(int)
    def __init__(self, parent=None):
        super().__init__()
        self.main = parent
        self.working = True
        self.count = 0
    def run(self):
        while self.working:
            self.count_changed.emit(self.count)
            self.sleep(1)
            self.count += 1

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    app.exec()