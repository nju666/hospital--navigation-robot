# from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton
# from PyQt5.QtCore import Qt, QUrl
# from PyQt5.QtGui import QFont
# from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage


# class AIPage(QWidget):
#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.parent = parent
#         self.init_ui()

#     def init_ui(self):
#         main_layout = QVBoxLayout(self)
#         main_layout.setContentsMargins(0, 0, 0, 0)

#         self.web_view = QWebEngineView()
#         # 加载网页
#         self.web_view.setUrl(QUrl("http://localhost:3000"))
#         # 网页加载完成后执行JavaScript代码
#         self.web_view.loadFinished.connect(self.on_webpage_loaded)
#         main_layout.addWidget(self.web_view, stretch=1)

#         # 悬浮返回按钮
#         self.back_btn = QPushButton("← 返回")
#         self.back_btn.setFixedSize(100, 40)
#         self.back_btn.setStyleSheet(
#             """
#             QPushButton {
#                 background-color: rgba(107, 153, 224, 0.8);
#                 color: white;
#                 border-radius: 6px;
#                 font-size: 16px;
#                 z-index: 9999;
#             }
#             QPushButton:hover {
#                 background-color: rgba(90, 136, 208, 0.8);
#             }
#         """
#         )
#         self.back_btn.clicked.connect(
#             lambda: self.parent.stacked_widget.setCurrentIndex(0)
#         )
#         self.back_btn.setParent(self)
#         self.back_btn.move(20, 20)
#         self.back_btn.raise_()

#     def on_webpage_loaded(self):
#         # 执行JavaScript代码请求麦克风权限并测试
#         js_code = """
#             navigator.mediaDevices.getUserMedia({ audio: true })
#                 .then(function(stream) {
#                     console.log('麦克风获取成功，流信息：', stream);
#                     // 这里可以进一步处理音频流，比如播放、录制等
#                     const audioContext = new (window.AudioContext || window.webkitAudioContext)();
#                     const source = audioContext.createMediaStreamSource(stream);
#                     source.connect(audioContext.destination);
#                 })
#                 .catch(function(err) {
#                     console.error('麦克风获取失败：', err);
#                 });
#         """
#         self.web_view.page().runJavaScript(js_code)


# 下面的有点回音，因为刚开始就打开了麦克风
# from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton
# from PyQt5.QtCore import Qt, QUrl
# from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage


# # 自定义页面以授权音频权限
# class CustomWebPage(QWebEnginePage):
#     def featurePermissionRequested(self, security_origin, feature):
#         if feature == QWebEnginePage.MediaAudioCapture:
#             print("请求麦克风权限")
#             self.setFeaturePermission(
#                 security_origin, feature, QWebEnginePage.PermissionGrantedByUser
#             )


# class AIPage(QWidget):
#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.parent = parent
#         self.init_ui()

#     def init_ui(self):
#         main_layout = QVBoxLayout(self)
#         main_layout.setContentsMargins(0, 0, 0, 0)

#         self.web_view = QWebEngineView()
#         self.web_view.setPage(CustomWebPage(self.web_view))  # 加这行！
#         self.web_view.setUrl(QUrl("http://localhost:3000"))
#         self.web_view.loadFinished.connect(self.on_webpage_loaded)
#         main_layout.addWidget(self.web_view, stretch=1)

#         self.back_btn = QPushButton("← 返回")
#         self.back_btn.setFixedSize(100, 40)
#         self.back_btn.setStyleSheet(
#             """
#             QPushButton {
#                 background-color: rgba(107, 153, 224, 0.8);
#                 color: white;
#                 border-radius: 6px;
#                 font-size: 16px;
#                 z-index: 9999;
#             }
#             QPushButton:hover {
#                 background-color: rgba(90, 136, 208, 0.8);
#             }
#         """
#         )
#         self.back_btn.clicked.connect(
#             lambda: self.parent.stacked_widget.setCurrentIndex(0)
#         )
#         self.back_btn.setParent(self)
#         self.back_btn.move(20, 20)
#         self.back_btn.raise_()

#     def on_webpage_loaded(self):
#         js_code = """
#             navigator.mediaDevices.getUserMedia({ audio: true })
#                 .then(function(stream) {
#                     console.log('麦克风获取成功，流信息：', stream);
#                     const audioContext = new (window.AudioContext || window.webkitAudioContext)();
#                     const source = audioContext.createMediaStreamSource(stream);
#                     source.connect(audioContext.destination);
#                 })
#                 .catch(function(err) {
#                     console.error('麦克风获取失败：', err);
#                 });
#         """
#         self.web_view.page().runJavaScript(js_code)


from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton
from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage


# 自定义页面以授权音频权限
class CustomWebPage(QWebEnginePage):
    def featurePermissionRequested(self, security_origin, feature):
        if feature == QWebEnginePage.MediaAudioCapture:
            print("请求麦克风权限")
            self.setFeaturePermission(
                security_origin, feature, QWebEnginePage.PermissionGrantedByUser
            )


class AIPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)

        self.web_view = QWebEngineView()
        self.web_view.setPage(CustomWebPage(self.web_view))
        self.web_view.setUrl(QUrl("http://localhost:3000"))
        self.web_view.loadFinished.connect(self.on_webpage_loaded)
        main_layout.addWidget(self.web_view, stretch=1)

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

    def on_webpage_loaded(self):
        # 修改后的JavaScript代码，只请求权限，不自动连接音频流
        js_code = """
            // 存储麦克风流和音频上下文
            window.microphoneStream = null;
            window.audioContext = null;
            
            // 初始化函数 - 只请求麦克风权限
            window.initAudio = function() {
                return navigator.mediaDevices.getUserMedia({ audio: true })
                    .then(function(stream) {
                        console.log('麦克风权限获取成功');
                        window.microphoneStream = stream;
                        // 立即停止流，防止自动播放
                        stream.getTracks().forEach(track => track.stop());
                        return true;
                    })
                    .catch(function(err) {
                        console.error('麦克风权限获取失败：', err);
                        return false;
                    });
            };
            
            // 开始通话函数 - 在用户点击拨号按钮后调用
            window.startCall = function() {
                if (!window.microphoneStream) {
                    // 如果流已停止，重新获取
                    return navigator.mediaDevices.getUserMedia({ audio: true })
                        .then(function(stream) {
                            console.log('重新获取麦克风流');
                            window.microphoneStream = stream;
                            
                            // 创建音频上下文，但不连接到扬声器（避免回环）
                            window.audioContext = new (window.AudioContext || window.webkitAudioContext)();
                            const source = window.audioContext.createMediaStreamSource(stream);
                            
                            // 这里应该连接到AI服务，而不是本地扬声器
                            console.log('开始通话，连接到AI服务...');
                            
                            return true;
                        })
                        .catch(function(err) {
                            console.error('重新获取麦克风失败：', err);
                            return false;
                        });
                }
                
                // 如果流存在但已停止，重新启动
                if (window.microphoneStream.getTracks()[0].readyState === 'ended') {
                    return window.initAudio().then(window.startCall);
                }
                
                // 创建音频上下文，但不连接到扬声器（避免回环）
                window.audioContext = new (window.AudioContext || window.webkitAudioContext)();
                const source = window.audioContext.createMediaStreamSource(window.microphoneStream);
                
                // 这里应该连接到AI服务，而不是本地扬声器
                console.log('开始通话，连接到AI服务...');
                
                return true;
            };
            
            // 结束通话函数
            window.endCall = function() {
                if (window.audioContext) {
                    window.audioContext.close();
                    window.audioContext = null;
                }
                
                if (window.microphoneStream) {
                    window.microphoneStream.getTracks().forEach(track => track.stop());
                    window.microphoneStream = null;
                }
                
                console.log('通话已结束');
            };
            
            // 页面加载完成后只初始化音频权限
            window.initAudio();
            
            // 监听页面中的拨号按钮点击事件（假设按钮有'dial-button'类）
            document.addEventListener('click', function(event) {
                if (event.target.classList.contains('dial-button')) {
                    window.startCall();
                }
            });
        """
        self.web_view.page().runJavaScript(js_code)
