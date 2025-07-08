from PyQt5.QtCore import QThread, pyqtSignal, QMutex, QWaitCondition


class CountdownThread(QThread):
    countdown_updated = pyqtSignal(str)  # 发送格式化后的字符串（如"09"）
    timeout = pyqtSignal()

    def __init__(self, total_seconds=30):
        super().__init__()
        self.total_seconds = total_seconds
        self.mutex = QMutex()
        self.condition = QWaitCondition()
        self.reset()

    def run(self):
        self.mutex.lock()
        while self.is_running and self.remaining_seconds > 0:
            self.condition.wait(self.mutex, 1000)
            if not self.is_running:
                break
            self.remaining_seconds -= 1
            self.countdown_updated.emit(
                f"{self.remaining_seconds:02d}"
            )  # 格式化为两位数字
        self.mutex.unlock()

        if self.remaining_seconds == 0:
            self.timeout.emit()

    def reset(self):
        self.mutex.lock()
        self.remaining_seconds = self.total_seconds
        self.is_running = True
        self.countdown_updated.emit(f"{self.remaining_seconds:02d}")  # 初始化时也格式化
        self.mutex.unlock()

    def stop(self):
        self.mutex.lock()
        self.is_running = False
        self.condition.wakeOne()
        self.mutex.unlock()
        self.wait()
