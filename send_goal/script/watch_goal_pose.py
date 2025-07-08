import time
import subprocess
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os

GOAL_FILE = "send_goal/goal_pose/goal_pose.txt"
LAUNCH_CMD = ["ros2", "launch", "send_goal", "send_goal.launch.py"]


class GoalPoseHandler(FileSystemEventHandler):
    def __init__(self, file_path):
        self.file_path = file_path
        self.last_modified = 0
        self.process = None  # 保存上一个进程

    def on_modified(self, event):
        if event.src_path != self.file_path:
            return

        now = time.time()
        if now - self.last_modified < 1:
            return  # 防止重复触发
        self.last_modified = now

        print(f"[INFO] 检测到 {self.file_path} 被修改，重新发送导航目标...")

        # 如果已有进程，先终止
        if self.process and self.process.poll() is None:
            print("[INFO] 终止上一次导航节点...")
            self.process.terminate()
            time.sleep(1)

        # 启动新的 send_goal 节点
        self.process = subprocess.Popen(LAUNCH_CMD)


def main():
    print(f"[INFO] 启动监听前清空 {GOAL_FILE} 内容...")
    try:
        with open(GOAL_FILE, "w") as f:
            f.write("")  # 清空文件内容
        print("[INFO] 文件已清空")
    except Exception as e:
        print(f"[ERROR] 无法清空目标文件: {e}")
        return

    print(f"[INFO] 监听 {GOAL_FILE} ...")
    event_handler = GoalPoseHandler(GOAL_FILE)
    observer = Observer()
    observer.schedule(event_handler, path=os.path.dirname(GOAL_FILE), recursive=False)
    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[INFO] 手动退出监听")
        observer.stop()
    observer.join()


if __name__ == "__main__":
    main()
