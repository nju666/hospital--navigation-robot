from PyQt5.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QStackedWidget,
    QPushButton,
    QMessageBox,
)
from PyQt5.QtCore import Qt, QTimer, QDateTime
from PyQt5.QtGui import QFont, QColor, QLinearGradient, QPalette, QBrush
from department_page import DepartmentPage
from voice_page import VoicePage
from ai_page import AIPage
from appointment_page import AppointmentPage
from doctor_page import DoctorPage
from doctor_page import DoctorPage, ConfirmPage 


class MedicalGuideSystem(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_timer()

    def init_ui(self):
        self.setWindowTitle("智能导医系统")
        self.setFixedSize(1000,500)
       # self.showFullScreen()
        #self.setFixedSize(1707, 960)
        self.show()
        self.init_palette()
        self.init_fonts()

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        main_layout = QVBoxLayout(self.central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        self.create_top_bar()
        main_layout.addWidget(self.top_bar)

        self.content_widget = QWidget()
        content_layout = QVBoxLayout(self.content_widget)
        content_layout.setContentsMargins(20, 20, 20, 20)

        self.stacked_widget = QStackedWidget()
        content_layout.addWidget(self.stacked_widget)

        main_layout.addWidget(self.content_widget, stretch=1)

        self.create_main_page()
        self.create_sub_pages()

        self.stacked_widget.setCurrentIndex(0)
        self.update_datetime()
        
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.showFullScreen()


    def init_palette(self):
        palette = QPalette()
        gradient = QLinearGradient(0, 0, 0, self.height())
        gradient.setColorAt(0, QColor(240, 247, 255))
        gradient.setColorAt(1, QColor(220, 235, 250))
        palette.setBrush(QPalette.Window, QBrush(gradient))
        self.setPalette(palette)

    def init_fonts(self):
        self.title_font = QFont("SimHei", 28, QFont.Bold)
        self.button_font = QFont("SimHei", 20)
        self.datetime_font = QFont("SimHei", 16)

    def create_top_bar(self):
        self.top_bar = QWidget()
        self.top_bar.setStyleSheet("background-color: #003399;")
        self.top_bar.setFixedHeight(70)

        bar_layout = QHBoxLayout(self.top_bar)
        bar_layout.setContentsMargins(20, 0, 20, 0)

        title_label = QLabel("智能导医系统")
        title_label.setFont(self.title_font)
        title_label.setStyleSheet("color: white;")
        bar_layout.addWidget(title_label, alignment=Qt.AlignLeft)

        self.datetime_label = QLabel()
        self.datetime_label.setFont(self.datetime_font)
        self.datetime_label.setStyleSheet("color: white;")
        bar_layout.addWidget(self.datetime_label, alignment=Qt.AlignRight)

    def create_main_page(self):
        main_page = QWidget()
        layout = QVBoxLayout(main_page)
        layout.setAlignment(Qt.AlignCenter)

        button_area = QWidget()
        button_layout = QHBoxLayout(button_area)
        button_layout.setSpacing(40)

        left_buttons = QVBoxLayout()
        left_buttons.setSpacing(30)

        dept_button = QPushButton("科室导航")
        dept_button.setFont(self.button_font)
        dept_button.setStyleSheet(
            """
            QPushButton {
                background-color: #4a86e8;
                color: white;
                border-radius: 15px;
                padding: 25px 40px;
                border: 2px solid #3a76d8;
            }
            QPushButton:hover {
                background-color: #3a76d8;
            }
        """
        )
        dept_button.setFixedSize(220, 90)
        dept_button.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(1))
        left_buttons.addWidget(dept_button)

        ai_button = QPushButton("智能问诊")
        ai_button.setFont(self.button_font)
        ai_button.setStyleSheet(
            """
            QPushButton {
                background-color: #67c23a;
                color: white;
                border-radius: 15px;
                padding: 25px 40px;
                border: 2px solid #52a62d;
            }
            QPushButton:hover {
                background-color: #52a62d;
            }
        """
        )
        ai_button.setFixedSize(220, 90)
        ai_button.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(3))
        left_buttons.addWidget(ai_button)

        right_buttons = QVBoxLayout()
        right_buttons.setSpacing(30)

        voice_button = QPushButton("语音导航")
        voice_button.setFont(self.button_font)
        voice_button.setStyleSheet(
            """
            QPushButton {
                background-color: #e6a23c;
                color: white;
                border-radius: 15px;
                padding: 25px 40px;
                border: 2px solid #d38b2f;
            }
            QPushButton:hover {
                background-color: #d38b2f;
            }
        """
        )
        voice_button.setFixedSize(220, 90)
        voice_button.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(2))
        right_buttons.addWidget(voice_button)

        appointment_button = QPushButton("预约挂号")
        appointment_button.setFont(self.button_font)
        appointment_button.setStyleSheet(
            """
            QPushButton {
                background-color: #f56c6c;
                color: white;
                border-radius: 15px;
                padding: 25px 40px;
                border: 2px solid #e05b5b;
            }
            QPushButton:hover {
                background-color: #e05b5b;
            }
        """
        )
        appointment_button.setFixedSize(220, 90)
        appointment_button.clicked.connect(
            lambda: self.stacked_widget.setCurrentIndex(4)
        )

        right_buttons.addWidget(appointment_button)

        button_layout.addLayout(left_buttons)
        button_layout.addLayout(right_buttons)

        layout.addWidget(button_area, alignment=Qt.AlignCenter)

        self.stacked_widget.addWidget(main_page)

    def create_sub_pages(self):
        self.dept_page = DepartmentPage(self)
        self.voice_page = VoicePage(self)
        self.ai_page = AIPage(self)
        self.appointment_page = AppointmentPage(self)

        self.stacked_widget.addWidget(self.dept_page)
        self.stacked_widget.addWidget(self.voice_page)
        self.stacked_widget.addWidget(self.ai_page)
        self.stacked_widget.addWidget(self.appointment_page)
        self.doctor_page = DoctorPage(self)
        self.stacked_widget.addWidget(self.doctor_page)
        self.doctor_page = DoctorPage(self)
        self.confirm_page = ConfirmPage(self)
        self.stacked_widget.addWidget(self.doctor_page)
        self.stacked_widget.addWidget(self.confirm_page)

        
    def init_timer(self):
        # 创建一个计时器，每秒触发一次更新
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_datetime)
        self.timer.start(1000)  # 1000毫秒 = 1秒

    def update_datetime(self):
        # 获取当前日期时间
        current_date_time = QDateTime.currentDateTime()

        # 获取星期几(1-7)，转换为汉字表示
        day_of_week = current_date_time.date().dayOfWeek()
        week_day_map = {
            1: "星期一",
            2: "星期二",
            3: "星期三",
            4: "星期四",
            5: "星期五",
            6: "星期六",
            7: "星期日",
        }
        week_day_str = week_day_map.get(day_of_week, "未知")

        # 格式化日期时间字符串，包含年月日、汉字星期和时分秒
        date_str = current_date_time.toString(f"yyyy年MM月dd日 {week_day_str} HH:mm:ss")

        # 将格式化后的字符串显示在标签上
        self.datetime_label.setText(date_str)


    def confirm_and_goto_appointment(self):
        reply = QMessageBox.question(
            self,
            "跳转提示",
            "是否前往预约挂号页面？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.stacked_widget.setCurrentIndex(4)
    def show_doctor_page(self, dept_name):
        self.doctor_page.set_doctors(dept_name)
        self.stacked_widget.setCurrentWidget(self.doctor_page)
    
        
    

