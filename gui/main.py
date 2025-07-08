#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets import QApplication
from main_window import MedicalGuideSystem
import os

os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--use-fake-ui-for-media-stream --no-sandbox"
os.environ["QTWEBENGINE_DISABLE_SANDBOX"] = "1"

if __name__ == "__main__":
    # 设置高DPI支持
    if hasattr(sys, "AA_EnableHighDpiScaling"):
        QApplication.setAttribute(sys.AA_EnableHighDpiScaling, True)
    if hasattr(sys, "AA_UseHighDpiPixmaps"):
        QApplication.setAttribute(sys.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    window = MedicalGuideSystem()
    window.show()
    sys.exit(app.exec_())
