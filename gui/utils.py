from datetime import datetime


def update_datetime():
    """获取当前时间并格式化为指定字符串"""
    now = datetime.now()
    # 格式：年-月-日 时:分:秒 星期X
    return (
        now.strftime("%Y年%m月%d日 %H:%M:%S %A")
        .replace("Monday", "一")
        .replace("Tuesday", "二")
        .replace("Wednesday", "三")
        .replace("Thursday", "四")
        .replace("Friday", "五")
        .replace("Saturday", "六")
        .replace("Sunday", "日")
    )
    