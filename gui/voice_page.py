from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QFrame,
    QSpacerItem,
    QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QFont, QMovie
import subprocess
import os
import logging
from PyQt5.QtCore import QProcess


class VoicePage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.init_ui()

        self.voice_process = None  # 新增属性初始化

        self.reset_state()
        

    def init_ui(self):
        # 使用固定布局
        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(20, 20, 20, 20)
        self.main_layout.setSpacing(0)

        # 悬浮返回按钮（模仿AIPage的实现）
        self.back_btn = QPushButton("← 返回")
        self.back_btn.setFixedSize(100, 40)
        self.back_btn.setStyleSheet(
            """
            QPushButton {
                background-color: rgba(107, 153, 224, 0.8);
                color: white;
                border-radius: 6px;
                font-size: 16px;
                z-index: 9999;
            }
            QPushButton:hover {
                background-color: rgba(90, 136, 208, 0.8);
            }
        """
        )
        self.back_btn.clicked.connect(
            lambda: self.parent.stacked_widget.setCurrentIndex(0)
        )
        self.back_btn.setParent(self)
        self.back_btn.move(20, 20)
        self.back_btn.raise_()

        # 顶部区域 - 仅保留标题和分割线
        top_container = QWidget()
        top_container.setFixedHeight(80)  # 调整高度以适应新布局
        self.main_layout.addWidget(top_container)

        top_layout = QVBoxLayout(top_container)

        title_label = QLabel("语音导航")
        title_label.setFont(QFont("SimHei", 32, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: #3a76d8;")
        top_layout.addWidget(title_label)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        top_layout.addWidget(line)

        # 为按钮区域创建固定高度的容器
        self.button_container = QWidget()
        button_layout = QVBoxLayout(self.button_container)
        button_layout.setContentsMargins(0, 20, 0, 20)

        # 添加垂直间隔，将按钮定位在上方
        button_layout.addSpacerItem(
            QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        )

        self.voice_button = QPushButton("启动语音识别")
        self.voice_button.setFont(QFont("SimHei", 24))
        self.voice_button.setFixedSize(280, 70)
        self.voice_button.setStyleSheet(
            """
            QPushButton {
                background-color: #4a86e8;
                color: white;
                border-radius: 10px;
                padding: 15px 30px;
            }
            QPushButton:hover {
                background-color: #3a76d8;
            }
        """
        )
        self.voice_button.clicked.connect(self.run_voice_script)

        # 居中放置按钮
        button_layout.addWidget(self.voice_button, alignment=Qt.AlignHCenter)

        # 添加垂直间隔，将按钮定位在上方
        button_layout.addSpacerItem(
            QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        )

        self.main_layout.addWidget(self.button_container, stretch=1)

        # 创建固定位置的GIF容器
        self.gif_container = QWidget()
        self.gif_container.setFixedSize(750, 300)
        self.gif_layout = QVBoxLayout(self.gif_container)
        self.gif_layout.setContentsMargins(0, 0, 0, 0)

        # GIF 1 - 启动中
        self.gif1_label = QLabel(self.gif_container)
        self.gif1_label.setAlignment(Qt.AlignCenter)
        self.gif1_label.setFixedSize(750, 300)
        self.gif1_label.setVisible(False)
        self.movie1 = QMovie("/root/gui/voice.gif")  # 替换为实际路径
        self.gif1_label.setMovie(self.movie1)
        self.gif_layout.addWidget(self.gif1_label, alignment=Qt.AlignCenter)

        # GIF 2 - 等待完成 (使用独立容器实现覆盖效果)
        self.gif2_container = QWidget(self)
        self.gif2_container.setFixedSize(600, 600)
        self.gif2_container.setVisible(False)

        self.gif2_label = QLabel(self.gif2_container)
        self.gif2_label.setAlignment(Qt.AlignCenter)
        self.gif2_label.setFixedSize(600, 600)
        self.gif2_label.setVisible(False)
        self.movie2 = QMovie("/root/gui/new_arrow.gif")  # 替换为实际路径
        self.gif2_label.setMovie(self.movie2)

        gif2_layout = QVBoxLayout(self.gif2_container)
        gif2_layout.addWidget(self.gif2_label, alignment=Qt.AlignCenter)

        self.main_layout.addWidget(self.gif_container, alignment=Qt.AlignCenter)

    def reset_state(self):
        """重置所有动态控件的状态到初始值"""
        self.voice_button.setEnabled(True)

        self.movie1.stop()
        self.gif1_label.setVisible(False)

        self.movie2.stop()
        self.gif2_label.setVisible(False)
        self.gif2_container.setVisible(False)

        # 终止语音进程，针对 QProcess 做适配
        if self.voice_process is not None:
            # 判断进程是否正在运行
            if self.voice_process.state() == QProcess.Running:  
                try:
                    self.voice_process.terminate()  # 尝试优雅终止
                    # 等待进程终止（可按需设置超时，这里简单处理）
                    self.voice_process.waitForFinished()  
                    self.voice_process = None
                    print("语音进程已终止")
                except Exception as e:
                    print(f"终止语音进程失败: {str(e)}")

        if hasattr(self, "done_check_timer") and self.done_check_timer.isActive():
            self.done_check_timer.stop()

        if hasattr(self, "initial_wait_timer") and self.initial_wait_timer.isActive():
            self.initial_wait_timer.stop()

    def run_voice_script(self):
        """启动语音识别脚本并显示第一个GIF"""
        self.reset_state()

        # 显示第一个 GIF (在按钮下方)
        self.gif1_label.setVisible(True)
        self.movie1.start()

        # 禁用按钮防止重复点击
        self.voice_button.setEnabled(False)

        # 启动脚本
        subprocess.Popen(["bash", "/root/voice_id/total.sh"])

        # 5秒后切换到第二个GIF
        QTimer.singleShot(12000, self.show_second_gif)

    def show_second_gif(self):
        """显示第二个GIF（覆盖按钮区域）"""
        # 停止第一个GIF
        self.movie1.stop()
        self.gif1_label.setVisible(False)

        # 计算GIF2的居中位置
        x = (self.width() - self.gif2_container.width()) // 2
        y = (self.height() - self.gif2_container.height()) // 2
        self.gif2_container.move(x, y)

        # 显示第二个GIF（覆盖布局）
        self.gif2_container.setVisible(True)
        self.gif2_label.setVisible(True)
        self.movie2.start()
        print("显示第二个GIF，准备播放开始语音")

        # 播放开始语音
        self.play_start_voice()

        # 15秒后开始检查文件
        self.initial_wait_timer = QTimer(self)
        self.initial_wait_timer.timeout.connect(self.start_done_file_check)
        self.initial_wait_timer.setSingleShot(True)
        self.initial_wait_timer.start(10000)
        print("启动 10 秒后检查文件的定时器")

    def start_done_file_check(self):
        """开始定时检查文件"""
        self.done_check_timer = QTimer(self)
        self.done_check_timer.timeout.connect(self.check_done_in_file)
        self.done_check_timer.start(500)  # 每500ms检查一次

    def check_done_in_file(self):
        """检查文件内容是否包含'done'"""
        file_path = (
            "/userdata/dev_ws/src/originbot/originbot_send_goal/goal_pose/goal_pose.txt"
        )
        try:
            if os.path.exists(file_path):
                with open(file_path, "r", encoding="utf-8") as f:
                    contents = f.read()
                    if "done" in contents:
                        self.done_check_timer.stop()
                        self.return_to_main()
        except Exception as e:
            print(f"检查文件时出错: {e}")

    def return_to_main(self):
        """返回主界面并重置状态"""

        # 停止第二个GIF
        if self.movie2 and self.movie2.state() == QMovie.Running:
            self.movie2.stop()
        self.gif2_label.setVisible(False)
        self.gif2_container.setVisible(False)

        # 播放完成语音
        self.play_finish_voice()

        self.reset_state()
        self.parent.stacked_widget.setCurrentIndex(0)


    def play_start_voice(self):
        """播放开始语音（显示第二个GIF时调用）"""
        print("开始执行 play_start_voice")  # 新增调试打印
        try:
            # 改用 QProcess 更适配 PyQt 事件循环，也方便捕获输出
            self.voice_process = QProcess(self)
            # 连接错误输出信号，实时打印错误
            self.voice_process.readyReadStandardError.connect(
                lambda: print(
                    "语音播放错误输出:", 
                    self.voice_process.readAllStandardError().data().decode('utf-8')
                )
            )
            # 启动播放命令
            self.voice_process.start(
                "tinyplay", 
                ["-D", "0", "-d", "0", "/root/gui/new_start.wav"]  # 替换实际音频路径
            )
            print("已调用 tinyplay 尝试播放开始语音")
        except Exception as e:
            print(f"播放开始语音失败: {str(e)}")
    
    def play_finish_voice(self):
        """播放完成语音（GIF停止显示时调用）"""
        print("开始执行 play_finish_voice")  # 新增调试打印
        try:
            # 终止可能存在的语音进程
            if self.voice_process is not None and self.voice_process.state() == QProcess.Running:
                self.voice_process.terminate()
                print("终止之前的语音进程")

            # 播放完成语音
            self.voice_process = QProcess(self)
            self.voice_process.readyReadStandardError.connect(
                lambda: print(
                    "完成语音播放错误输出:", 
                    self.voice_process.readAllStandardError().data().decode('utf-8')
                )
            )
            self.voice_process.start(
                "tinyplay", 
                ["-D", "0", "-d", "0", "/root/gui/new_finish.wav"]  # 替换实际音频路径
            )
            print("已调用 tinyplay 尝试播放完成语音")
        except Exception as e:
            print(f"播放完成语音失败: {str(e)}")