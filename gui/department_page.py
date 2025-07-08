from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QGridLayout,
    QFrame,
    QMessageBox,
    QSpacerItem,
    QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QEvent
from PyQt5.QtGui import QFont, QColor, QMovie
import os
import logging
import subprocess
import signal
from dept_config import DEPT_COORDS
import time


class DepartmentPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.init_ui()
        self.listener_process = None  # 初始化进程对象
        self.done_check_timer = None  # 初始化定时器对象

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)

        top_layout = QHBoxLayout()

        back_button = QPushButton("← 返回")
        back_button.setFixedSize(120, 50)
        back_button.setStyleSheet(
            """
            QPushButton {
                background-color: #6b99e0;
                color: white;
                border-radius: 8px;
                font-size: 18px;
            }
            QPushButton:hover {
                background-color: #5a88d0;
            }
            """
        )
        back_button.clicked.connect(self.return_to_main)
        top_layout.addWidget(back_button, alignment=Qt.AlignLeft)

        title_label = QLabel("请选择你要去的科室")
        title_label.setFont(QFont("SimHei", 32, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: #3a76d8;")
        top_layout.addWidget(title_label, stretch=1)

        self.countdown_label = QLabel("30秒")
        self.countdown_label.setFont(QFont("SimHei", 20, QFont.Bold))
        self.countdown_label.setStyleSheet("color: red;")
        top_layout.addWidget(self.countdown_label, alignment=Qt.AlignRight)

        layout.addLayout(top_layout)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

        self.dept_grid = QGridLayout()
        self.dept_grid.setSpacing(20)
        self.dept_grid.setAlignment(Qt.AlignCenter)

        self.departments = [
            "内科",
            "外科",
            "妇产科",
            "儿科",
            "眼科",
            "耳鼻喉科",
            "口腔科",
            "皮肤科",
            "神经内科",
        ]
        self.current_page = 0
        self.page_size = 6
        self.total_pages = (
            len(self.departments) + self.page_size - 1
        ) // self.page_size

        page_layout = QHBoxLayout()
        page_layout.setAlignment(Qt.AlignCenter)

        self.prev_page_button = QPushButton("上一页")
        self.prev_page_button.setFixedSize(120, 40)
        self.prev_page_button.setStyleSheet(
            """
            QPushButton {
                background-color: #f5f5f5;
                color: #333;
                border-radius: 8px;
                border: 1px solid #ddd;
            }
            QPushButton:hover {
                background-color: #e5e5e5;
            }
        """
        )
        self.prev_page_button.clicked.connect(self.prev_page)

        self.next_page_button = QPushButton("下一页")
        self.next_page_button.setFixedSize(120, 40)
        self.next_page_button.setStyleSheet(
            """
            QPushButton {
                background-color: #f5f5f5;
                color: #333;
                border-radius: 8px;
                border: 1px solid #ddd;
            }
            QPushButton:hover {
                background-color: #e5e5e5;
            }
        """
        )
        self.next_page_button.clicked.connect(self.next_page)

        page_layout.addWidget(self.prev_page_button)
        page_layout.addWidget(self.next_page_button)

        layout.addLayout(self.dept_grid)
        layout.addLayout(page_layout)

        self.load_department_page(self.current_page)

    def load_department_page(self, page_num):
        for i in reversed(range(self.dept_grid.count())):
            widget = self.dept_grid.itemAt(i).widget()
            if widget:
                widget.deleteLater()

        start_idx = page_num * self.page_size
        end_idx = min(start_idx + self.page_size, len(self.departments))
        current_departments = self.departments[start_idx:end_idx]

        row, col = 0, 0
        for dept in current_departments:
            dept_button = QPushButton(dept)
            dept_button.setFont(QFont("SimHei", 18))
            dept_button.setStyleSheet(
                """
                QPushButton {
                    background-color: #e0f3ff;
                    color: #333;
                    border-radius: 10px;
                    padding: 15px 30px;
                    border: 2px solid #b3d9f9;
                }
                QPushButton:hover {
                    background-color: #b3d9f9;
                }
            """
            )
            dept_button.setFixedSize(200, 60)
            dept_button.clicked.connect(lambda _, d=dept: self.select_department(d))
            self.dept_grid.addWidget(dept_button, row, col)

            col += 1
            if col >= 3:
                col = 0
                row += 1

        self.prev_page_button.setEnabled(page_num > 0)
        self.next_page_button.setEnabled(page_num < self.total_pages - 1)

    def select_department(self, department):
        if hasattr(self, "countdown_thread"):
            self.countdown_thread.stop()
        self.show_confirm_widget(department)

    def show_confirm_widget(self, department):
        self.confirm_widget = QWidget(self)
        self.confirm_widget.setFixedSize(400, 200)
        self.confirm_widget.setStyleSheet(
            "background-color: white; border-radius: 10px; border: 2px solid #3a76d8;"
        )

        layout = QVBoxLayout(self.confirm_widget)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)

        title_label = QLabel("确认科室选择")
        title_label.setFont(QFont("SimHei", 20, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: #3a76d8;")
        layout.addWidget(title_label)

        content_label = QLabel(f"你选择的科室是：{department}")
        content_label.setFont(QFont("SimHei", 16))
        content_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(content_label)

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(20)
        btn_layout.setAlignment(Qt.AlignCenter)

        cancel_btn = QPushButton("取消")
        cancel_btn.setFont(QFont("SimHei", 16))
        cancel_btn.setStyleSheet(
            """
            QPushButton {
                background-color: #f5f5f5;
                color: #333;
                border-radius: 8px;
                padding: 10px 25px;
                border: 1px solid #ddd;
            }
            QPushButton:hover {
                background-color: #e5e5e5;
            }
        """
        )
        cancel_btn.setFixedSize(120, 45)
        cancel_btn.clicked.connect(self.hide_confirm_widget)
        btn_layout.addWidget(cancel_btn)

        confirm_btn = QPushButton("确认")
        confirm_btn.setFont(QFont("SimHei", 16))
        confirm_btn.setStyleSheet(
            """
            QPushButton {
                background-color: #4a86e8;
                color: white;
                border-radius: 8px;
                padding: 10px 25px;
                border: 1px solid #3a76d8;
            }
            QPushButton:hover {
                background-color: #3a76d8;
            }
        """
        )
        confirm_btn.setFixedSize(120, 45)
        confirm_btn.clicked.connect(lambda: self.confirm_selection(department))
        btn_layout.addWidget(confirm_btn)

        layout.addLayout(btn_layout)

        self.confirm_widget.move(
            (self.width() - self.confirm_widget.width()) // 2,
            (self.height() - self.confirm_widget.height()) // 2,
        )
        self.confirm_widget.show()

    def hide_confirm_widget(self):
        if hasattr(self, "confirm_widget") and self.confirm_widget:
            self.confirm_widget.deleteLater()
            self.confirm_widget = None
        if hasattr(self, "countdown_thread"):
            self.countdown_thread.start()

    def confirm_selection(self, department):
        if hasattr(self, "confirm_widget") and self.confirm_widget:
            self.confirm_widget.deleteLater()
            self.confirm_widget = None

        self.show_loading_overlay()

        # 获取科室坐标
        coords = DEPT_COORDS.get(department, (0.0, 0.0))

        # 启动监听进程
        # self.start_listener_process()

        # time.sleep(1)

        # 写入坐标到文件
        self.write_coords_to_file(coords)

        # 开始检查文件
        self.start_done_file_check()

    def return_to_main_with_hide(self):
        self.hide_loading_overlay()
        self.return_to_main()

    def write_coords_to_file(self, coords):
        file_path = (
            "/userdata/dev_ws/src/originbot/originbot_send_goal/goal_pose/goal_pose.txt"
        )
        try:
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(f"{coords[0]} {coords[1]}")
            logging.info(f"成功写入坐标：{coords} 到文件 {file_path}")
        except Exception as e:
            logging.error(f"写入坐标失败：{str(e)}，文件路径：{file_path}")

    def return_to_main(self):
        self.parent.stacked_widget.setCurrentIndex(0)

    def prev_page(self):
        if self.current_page > 0:
            self.current_page -= 1
            self.load_department_page(self.current_page)

    def next_page(self):
        if self.current_page < self.total_pages - 1:
            self.current_page += 1
            self.load_department_page(self.current_page)

    def update_countdown_display(self, remaining):
        self.countdown_label.setText(f"{remaining}秒")

    def countdown_timeout(self):
        self.return_to_main()

    def show_loading_overlay(self):
        self.loading_overlay = QWidget(self)
        self.loading_overlay.setStyleSheet(
            "background-color: rgba(255, 255, 255, 200);"
        )
        self.loading_overlay.setGeometry(0, 0, self.width(), self.height())

        layout = QVBoxLayout(self.loading_overlay)
        layout.setAlignment(Qt.AlignCenter)

        self.loading_label = QLabel()
        self.loading_label.setFixedSize(800, 600)
        self.loading_label.setAlignment(Qt.AlignCenter)

        self.loading_movie = QMovie("new_arrow.gif")
        self.loading_label.setMovie(self.loading_movie)

        # 连接GIF播放完成信号（如果GIF是循环播放，此信号在每次循环结束时触发）
        self.loading_movie.finished.connect(self.on_gif_finished)

        self.loading_movie.start()

        layout.addWidget(self.loading_label)
        self.loading_overlay.show()

        # 播放开始语音
        self.play_start_sound()

    def hide_loading_overlay(self):
        if hasattr(self, "loading_overlay") and self.loading_overlay:
            self.loading_movie.stop()

            # 断开信号连接，避免重复触发
            self.loading_movie.finished.disconnect(self.on_gif_finished)

            self.loading_overlay.deleteLater()
            self.loading_overlay = None

            # 播放结束语音
            self.play_finish_sound()

    def start_listener_process(self):
        """启动监听进程"""
        if self.listener_process is not None and self.listener_process.poll() is None:
            # 已有运行中的进程，先终止它
            self.terminate_listener_process()

        # 拼接要执行的命令（ROS 环境需要先 source 再执行 Python 脚本）
        bash_command = (
            "source /opt/ros/humble/setup.bash && "
            "source /userdata/dev_ws/install/setup.bash && "
            "python3 /userdata/dev_ws/src/originbot/originbot_send_goal/script/watch_goal_pose.py"
        )

        try:
            # 启动子进程，用 shell 模式执行命令，同时创建新会话方便后续终止
            self.listener_process = subprocess.Popen(
                bash_command,
                shell=True,
                executable="/bin/bash",
                start_new_session=True,  # 让进程组独立，方便批量终止
                # stdout=subprocess.PIPE,  # 可根据需要决定是否捕获输出
                # stderr=subprocess.STDOUT,
            )
            logging.info(f"监听子进程已启动，PID: {self.listener_process.pid}")
        except Exception as e:
            logging.error(f"启动监听进程失败: {str(e)}")
            self.listener_process = None

    def terminate_listener_process(self):
        """终止监听进程"""
        if self.listener_process is None:
            return

        try:
            # 先尝试优雅终止（发送 SIGTERM 信号）
            os.killpg(os.getpgid(self.listener_process.pid), signal.SIGTERM)
            logging.info(
                f"已向进程组 {os.getpgid(self.listener_process.pid)} 发送 SIGTERM 信号"
            )

            # 等待进程退出，最多等 5 秒
            self.listener_process.wait(timeout=5)
            logging.info("监听进程已正常终止")
        except subprocess.TimeoutExpired:
            # 超时则强制杀死（发送 SIGKILL 信号）
            try:
                os.killpg(os.getpgid(self.listener_process.pid), signal.SIGKILL)
                logging.warning("进程未及时响应，已强制终止")
            except Exception as e:
                logging.error(f"强制终止进程失败: {str(e)}")
        except Exception as e:
            logging.error(f"终止进程时发生异常: {str(e)}")
        finally:
            # 无论如何，清空进程对象
            self.listener_process = None
            self.done_check_timer = None

    def start_done_file_check(self):
        """开始检查文件中是否包含'done'"""
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
                        # 停止定时器
                        if self.done_check_timer:
                            self.done_check_timer.stop()

                        # 终止监听进程
                        self.terminate_listener_process()

                        # 隐藏加载动画并返回主界面
                        self.hide_loading_overlay()
                        self.return_to_main()
        except Exception as e:
            logging.error(f"检查文件时出错: {str(e)}")

    def play_start_sound(self):
        """播放开始语音"""
        try:
            # 使用subprocess异步播放语音，避免阻塞UI
            subprocess.Popen(
                ["tinyplay", "-D", "1", "-d", "0", "new_start.wav"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=False
            )
            logging.info("开始语音播放")
        except Exception as e:
            logging.error(f"播放开始语音失败: {str(e)}")

    def on_gif_finished(self):
        """GIF播放完成时的回调（如果是循环播放，每次循环结束都会触发）"""
        logging.info("GIF播放完成或循环结束")
        # 如果需要在GIF完全停止时执行操作，可在此添加逻辑

    def play_finish_sound(self):
        """播放结束语音"""
        try:
            subprocess.Popen(
                ["tinyplay", "-D", "1", "-d", "0", "new_finish.wav"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=False
            )
            logging.info("结束语音播放")
        except Exception as e:
            logging.error(f"播放结束语音失败: {str(e)}")
