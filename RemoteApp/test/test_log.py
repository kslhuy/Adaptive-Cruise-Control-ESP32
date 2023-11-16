import sys
import time
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QTextEdit

class WorkerThread(QThread):
    log_signal = pyqtSignal(str)

    def run(self):
        for i in range(1, 6):
            self.log_signal.emit(f"Log message {i}")
            time.sleep(1)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Thread Communication Example")
        self.setGeometry(100, 100, 800, 600)

        layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)

        self.start_button = QPushButton("Start Worker Thread")
        self.start_button.clicked.connect(self.start_worker_thread)
        layout.addWidget(self.start_button)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def start_worker_thread(self):
        self.worker_thread = WorkerThread()
        self.worker_thread.log_signal.connect(self.update_log)
        self.worker_thread.start()

    def update_log(self, message):
        self.log_text.append(message)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
