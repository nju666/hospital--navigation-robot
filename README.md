# Intelligent Medical Guide Robot

## Graphical User Interface

The `gui` directory contains the graphical user interface program, which is based on the PyQt5 framework to realize human-computer interaction. Enter the directory and run `python3 main.py` to start it.

The structure of the `gui` directory is as follows:

```
gui/
├── main.py                # Program entry point
├── main_window.py         # Main window class
├── department_page.py     # Department navigation page
├── voice_page.py          # Voice consultation page
├── ai_page.py             # Intelligent consultation page
├── appointment_page.py    # Appointment registration page
├── countdown_thread.py    # Countdown thread
├── resources/             # Resource folder (icons, styles, etc.)
└── utils.py               # Utility functions
```

## Speech Recognition

The content in the `voice_id` directory implements speech recognition combined with large model understanding. The project uses a Waveshare sound card. For specific usage methods, please refer to <https://d-robotics.github.io/rdk_doc/Basic_Application/audio/rdk_x5/audio_driver_hat2_rev2/>. Enter the directory and run `bash total.sh` to start it.

## Navigation Related

`cruising`, `face_identify`, `navigation`, and `send_goal` are all ROS nodes. When using them, add and compile them, then you can run them using ROS commands. `cruising` realizes robot patrol, `face_identify` realizes face recognition, `navigation` realizes SLAM map construction and navigation, and `send_goal` realizes sending target points.
