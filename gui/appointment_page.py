from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QFrame, QSpacerItem, QSizePolicy, QGridLayout
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QPalette, QLinearGradient, QColor, QBrush


class AppointmentPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.init_ui()

    def init_ui(self):
        self.setAutoFillBackground(True)
        palette = QPalette()
        gradient = QLinearGradient(0, 0, 0, self.height())
        gradient.setColorAt(0.0, QColor(240, 248, 255))  # AliceBlue
        gradient.setColorAt(1.0, QColor(224, 238, 255))  # Light blue
        palette.setBrush(QPalette.Window, QBrush(gradient))
        self.setPalette(palette)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(60, 30, 60, 30)
        layout.setSpacing(20)

        # 顶部返回按钮
        top_layout = QHBoxLayout()
        back_button = QPushButton("← 返回主页")
        back_button.setFixedSize(140, 50)
        back_button.setStyleSheet("""
            QPushButton {
                background-color: #6b99e0;
                color: white;
                border-radius: 8px;
                font-size: 18px;
            }
            QPushButton:hover {
                background-color: #5a88d0;
            }
        """)
        back_button.clicked.connect(lambda: self.parent.stacked_widget.setCurrentIndex(0))
        top_layout.addWidget(back_button, alignment=Qt.AlignLeft)
        layout.addLayout(top_layout)

        # 标题 + 副标题（组合在一起）
        header_widget = QWidget()
        header_layout = QVBoxLayout(header_widget)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(10)

        title = QLabel("欢迎使用预约挂号服务")
        title.setFont(QFont("SimHei", 30, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #2c3e50;")
        header_layout.addWidget(title)

        subtitle = QLabel("请选择要预约的科室")
        subtitle.setFont(QFont("SimHei", 20))
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet("color: #666;")
        header_layout.addWidget(subtitle)

        layout.addWidget(header_widget)

        # 分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

        # 科室按钮区域（可拉伸）
        dept_widget = QWidget()
        dept_layout = QVBoxLayout(dept_widget)
        dept_layout.setContentsMargins(0, 0, 0, 0)

        dept_grid = QGridLayout()
        dept_grid.setSpacing(40)
        departments = ["内科", "外科", "儿科", "妇科", "骨科", "皮肤科", "眼科", "口腔科"]
        for i, dept in enumerate(departments):
            btn = QPushButton(dept)
            btn.setFixedSize(180, 80)
            btn.setFont(QFont("SimHei", 20))
            btn.setStyleSheet("""
                QPushButton {
                    background-color: qlineargradient(
                        x1:0, y1:0, x2:1, y2:0,
                        stop:0 #409EFF, stop:1 #66b1ff
                    );
                    color: white;
                    border-radius: 12px;
                }
                QPushButton:hover {
                    background-color: #5aa9ff;
                }
            """)
            btn.clicked.connect(lambda _, d=dept: self.goto_doctor_page(d))
            dept_grid.addWidget(btn, i // 4, i % 4, alignment=Qt.AlignCenter)

        dept_layout.addLayout(dept_grid)
        layout.addWidget(dept_widget, stretch=1)  # 关键：让按钮区域自动扩展占据中间空间

        # 页面底部空白
        layout.addSpacerItem(QSpacerItem(20, 30, QSizePolicy.Minimum, QSizePolicy.Expanding))

    def goto_doctor_page(self, dept_name):
        self.parent.show_doctor_page(dept_name)
