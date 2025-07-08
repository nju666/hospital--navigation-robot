import os
import time
import uuid
import json
import requests
import tos
import argparse

# 解析命令行参数（仅保留本地文件路径）
parser = argparse.ArgumentParser(description='本地语音文件上传TOS并进行语音识别')
parser.add_argument('--input', required=True, help='本地语音文件路径（如：/root/audio.mp3）')
args = parser.parse_args()

# TOS配置（固定参数）
ak = "***"
sk = "***"
endpoint = "***"
region = "***"
bucket_name = "***"
object_key = "***"  # 固定对象名称
file_name = args.input      # 从命令行获取本地文件路径

# 语音识别配置（固定参数）
appid = "***"
token = "***"

# ---------------- TOS上传模块 ----------------
try:
    # 创建TOS客户端并上传文件
    client = tos.TosClientV2(ak, sk, endpoint, region)
    client.put_object_from_file(bucket_name, object_key, file_name)
    print(f"✅ 本地文件 {file_name} 已成功上传至TOS")
    file_url = f"https://{bucket_name}.{endpoint}/{object_key}"
except tos.exceptions.TosClientError as e:
    print(f"❌ TOS上传失败（客户端错误）: {e.message}")
    exit(1)
except tos.exceptions.TosServerError as e:
    print(f"❌ TOS上传失败（服务器错误）: {e.code} - {e.message}")
    print(f"错误请求ID: {e.request_id}")
    exit(1)
except Exception as e:
    print(f"❌ TOS上传失败（未知错误）: {e}")
    exit(1)

# ---------------- 语音识别提交任务模块 ----------------
def submit_task():
    submit_url = "https://openspeech.bytedance.com/api/v3/auc/bigmodel/submit"
    task_id = str(uuid.uuid4())
    
    headers = {
        "X-Api-App-Key": appid,
        "X-Api-Access-Key": token,
        "X-Api-Resource-Id": "volc.bigasr.auc",
        "X-Api-Request-Id": task_id,
        "X-Api-Sequence": "-1"
    }

    request = {
        "user": {"uid": "fake_uid"},
        "audio": {
            "url": file_url,
            "format": "mp3",
            "codec": "raw",
            "rate": 48000,
            "bits": 16,
            "channel": 2
        },

        "request": {
            "model_name": "bigmodel",
            "show_utterances": True
        }
    }
    
    print(f"✅ 提交语音识别任务，任务ID: {task_id}")
    response = requests.post(submit_url, data=json.dumps(request), headers=headers)
    
    if response.headers.get("X-Api-Status-Code") == "20000000":
        print(f"✅ 任务提交成功，状态码: {response.headers['X-Api-Status-Code']}")
        x_tt_logid = response.headers.get("X-Tt-Logid", "未获取到日志ID")
        print(f"任务日志ID: {x_tt_logid}\n")
        return task_id, x_tt_logid
    else:
        print(f"❌ 任务提交失败，响应头: {response.headers}")
        exit(1)

# ---------------- 语音识别查询任务模块 ----------------
def query_task(task_id, x_tt_logid):
    query_url = "https://openspeech.bytedance.com/api/v3/auc/bigmodel/query"
    
    headers = {
        "X-Api-App-Key": appid,
        "X-Api-Access-Key": token,
        "X-Api-Resource-Id": "volc.bigasr.auc",
        "X-Api-Request-Id": task_id,
        "X-Tt-Logid": x_tt_logid
    }

    response = requests.post(query_url, json.dumps({}), headers=headers)
    
    if "X-Api-Status-Code" in response.headers:
        print(f"ℹ️ 查询任务状态，状态码: {response.headers['X-Api-Status-Code']}")
        print(f"任务消息: {response.headers.get('X-Api-Message', '无消息')}\n")
        return response
    else:
        print(f"❌ 查询任务失败，响应头: {response.headers}")
        exit(1)

# ---------------- 主流程控制 ----------------
try:
    task_id, x_tt_logid = submit_task()
    
    print("⏳ 等待语音识别结果... (每1秒查询一次)")
    while True:
        query_response = query_task(task_id, x_tt_logid)
        code = query_response.headers.get("X-Api-Status-Code", "")
        
        if code == "20000000":  # 任务完成
            result_text = query_response.json()["result"]["text"]
            print("\n✅ 语音识别完成！结果如下：")
            print(result_text)
            exit(0)
        elif code not in ("20000001", "20000002"):  # 任务失败
            print("\n❌ 语音识别任务失败，请检查配置或文件格式")
            exit(1)
        
        time.sleep(1)  # 任务进行中，等待1秒后重试

except KeyboardInterrupt:
    print("\n⏹️ 程序被用户中断")
except Exception as e:
    print(f"\n❌ 运行时错误: {str(e)}")
    exit(1)