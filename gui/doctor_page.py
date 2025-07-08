# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel, QSpacerItem, QSizePolicy
from PyQt5.QtGui import QFont, QPalette, QColor, QLinearGradient, QBrush
from PyQt5.QtCore import Qt, QPropertyAnimation, QEasingCurve


class DoctorPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent

        # 背景渐变色
        palette = QPalette()
        gradient = QLinearGradient(0, 0, 0, self.height())
        gradient.setColorAt(0, QColor(230, 245, 255))
        gradient.setColorAt(1, QColor(200, 230, 250))
        palette.setBrush(QPalette.Window, QBrush(gradient))
        self.setAutoFillBackground(True)
        self.setPalette(palette)

        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(60, 40, 60, 40)
        self.layout.setSpacing(25)
        self.layout.setAlignment(Qt.AlignTop | Qt.AlignHCenter)

        self.title = QLabel("医生列表")
        self.title.setFont(QFont("SimHei", 32, QFont.Bold))
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet("""
            color: #2c3e50;
            text-shadow: 1px 1px 3px rgba(0,0,0,0.2);
        """)
        self.layout.addWidget(self.title)

        self.back_btn = QPushButton("← 返回科室选择")
        self.back_btn.setFixedSize(220, 60)
        self.back_btn.setStyleSheet("""
            QPushButton {
                background-color: #5a86d9;
                color: white;
                border-radius: 12px;
                font-size: 20px;
                font-weight: bold;
                padding: 10px;
                box-shadow: 3px 3px 6px rgba(0, 0, 0, 0.15);
                transition: all 0.3s ease;
            }
            QPushButton:hover {
                background-color: #3a6bd8;
                box-shadow: 5px 5px 12px rgba(0, 0, 0, 0.3);
            }
        """)
        self.back_btn.clicked.connect(self.go_back)
        self.layout.addWidget(self.back_btn, alignment=Qt.AlignLeft)

    def set_doctors(self, dept_name):
        # 清除旧按钮（保留标题和返回按钮）
        while self.layout.count() > 2:
            widget = self.layout.takeAt(2).widget()
            if widget:
                widget.deleteLater()

        self.title.setText(f"{dept_name} 医生列表")

        doctor_list = [f"{dept_name} - 张医生", f"{dept_name} - 李医生", f"{dept_name} - 王医生"]

        # 增加顶部和底部间隔
        self.layout.addSpacerItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

        for doctor in doctor_list:
            btn = QPushButton(doctor)
            btn.setFixedSize(280, 80)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #67C23A;
                    color: white;
                    border-radius: 15px;
                    font-size: 22px;
                    font-weight: 600;
                    box-shadow: 2px 4px 8px rgba(0, 0, 0, 0.15);
                    padding: 12px;
                    transition: all 0.25s ease;
                }
                QPushButton:hover {
                    background-color: #5ba430;
                    box-shadow: 4px 6px 12px rgba(0, 0, 0, 0.3);
                }
            """)
            btn.clicked.connect(lambda _, d=doctor: self.parent.confirm_page.set_doctor(d))
            self.layout.addWidget(btn, alignment=Qt.AlignHCenter)

        self.layout.addSpacerItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

    def go_back(self):
        self.parent.stacked_widget.setCurrentIndex(4)  # 回到预约挂号页面


class ConfirmPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent

        palette = QPalette()
        gradient = QLinearGradient(0, 0, 0, self.height())
        gradient.setColorAt(0, QColor(255, 250, 245))
        gradient.setColorAt(1, QColor(245, 235, 225))
        palette.setBrush(QPalette.Window, QBrush(gradient))
        self.setAutoFillBackground(True)
        self.setPalette(palette)

        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(60, 40, 60, 40)
        self.layout.setSpacing(30)
        self.layout.setAlignment(Qt.AlignTop | Qt.AlignHCenter)

        self.title = QLabel("")
        self.title.setFont(QFont("SimHei", 30, QFont.Bold))
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setWordWrap(True)
        self.title.setStyleSheet("color: #7d3f3f;")
        self.layout.addWidget(self.title)

        self.confirm_btn = QPushButton("确认挂号")
        self.confirm_btn.setFixedSize(240, 70)
        self.confirm_btn.setStyleSheet("""
            QPushButton {
                background-color: #67C23A;
                color: white;
                border-radius: 15px;
                font-size: 22px;
                font-weight: 700;
                padding: 14px;
                box-shadow: 3px 5px 10px rgba(0, 0, 0, 0.15);
                transition: all 0.3s ease;
            }
            QPushButton:hover {
                background-color: #569728;
                box-shadow: 4px 7px 15px rgba(0, 0, 0, 0.3);
            }
        """)
        self.confirm_btn.clicked.connect(self.confirm)
        self.layout.addWidget(self.confirm_btn, alignment=Qt.AlignCenter)

        self.back_btn = QPushButton("← 返回医生列表")
        self.back_btn.setFixedSize(240, 70)
        self.back_btn.setStyleSheet("""
            QPushButton {
                background-color: #f56c6c;
                color: white;
                border-radius: 15px;
                font-size: 22px;
                font-weight: 700;
                padding: 14px;
                box-shadow: 3px 5px 10px rgba(0, 0, 0, 0.15);
                transition: all 0.3s ease;
            }
            QPushButton:hover {
                background-color: #d24c4c;
                box-shadow: 4px 7px 15px rgba(0, 0, 0, 0.3);
            }
        """)
        self.back_btn.clicked.connect(self.back)
        self.layout.addWidget(self.back_btn, alignment=Qt.AlignCenter)

    def set_doctor(self, doctor_name):
        self.doctor_name = doctor_name
        self.title.setText(f"是否确认挂号：\n\n{doctor_name}？")
        self.confirm_btn.show()
        self.back_btn.setText("← 返回医生列表")
        self.back_btn.clicked.disconnect()
        self.back_btn.clicked.connect(self.back)
        self.parent.stacked_widget.setCurrentWidget(self)

    def confirm(self):
        self.title.setText(f"您已成功挂号：\n\n{self.doctor_name}")
        self.confirm_btn.hide()
        self.back_btn.setText("← 返回主页")
        self.back_btn.clicked.disconnect()
        self.back_btn.clicked.connect(self.go_home)

    def back(self):
        self.parent.stacked_widget.setCurrentWidget(self.parent.doctor_page)

    def go_home(self):
        self.parent.stacked_widget.setCurrentIndex(0)
