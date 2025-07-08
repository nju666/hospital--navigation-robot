# 智能导医机器人

## 图形界面

`gui`目录下是图形用户界面程序，基于 PyQt5 框架，用以实现人机交互。进入目录使用`python3 main.py`即可运行。

`gui`目录的结构如下：

```
gui/
├── main.py                # 程序入口点
├── main_window.py         # 主窗口类
├── department_page.py     # 科室导航页面
├── voice_page.py          # 语音问诊页面
├── ai_page.py             # 智能问诊页面
├── appointment_page.py    # 预约挂号页面
├── countdown_thread.py    # 倒计时线程
├── resources/             # 资源文件夹(图标、样式等)
└── utils.py               # 工具函数
```

## 语音识别

`voice_id`目录中的内容实现了语音识别加大模型理解。项目中使用的是微雪声卡，具体使用方法可查看<https://d-robotics.github.io/rdk_doc/Basic_Application/audio/rdk_x5/audio_driver_hat2_rev2/>。进入目录使用`bash total.sh`即可运行。

## 导航相关

`cruising`、`face_identify`、`navigation`和`send_goal`均为 ros 节点，使用时将其添加并编译，即可使用 ROS 命令运行。`cruising`实现机器人巡航，`face_identify`实现人脸识别，`navigation`实现 SLAM 地图构建与导航，`send_goal`实现发送目标点。
