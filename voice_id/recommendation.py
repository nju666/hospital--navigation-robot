import argparse
from openai import OpenAI
from ament_index_python.packages import get_package_share_directory
import os

# 诊室位置映射表
DEPARTMENT_COORDINATES = {
    "眼科": (-4.5, 0.9),
    "妇产科": (1.2, 0.7),
    "耳鼻喉科": (-3.2, 0.9),
    "口腔科": (-9.6, 0.0),
    "骨科": (-0.4, 0.2),
    "皮肤科": (3.2, 4.5),
    "内科": (2.0,0.2),
    "中医科": (-24.0,12.0),
    # 添加更多科室坐标
}

# 初始化 OpenAI 客户端（火山豆包）
client = OpenAI(
    base_url="***",
    api_key="***",  # 请替换为你自己的密钥
)

# 科室列表
DEPARTMENTS = [
    "内科",
    "外科",
    "妇产科",
    "儿科",
    "眼科",
    "耳鼻喉科",
    "口腔科",
    "皮肤科",
    "神经内科",
    "心血管内科",
    "消化内科",
    "呼吸内科",
    "内分泌科",
    "血液科",
    "肾内科",
    "感染科",
    "肿瘤科",
    "骨科",
    "泌尿外科",
    "神经外科",
    "心胸外科",
    "普外科",
    "整形外科",
    "急诊科",
    "康复科",
    "精神心理科",
    "中医科",
    "针灸科",
    "推拿科",
    "男科",
    "老年病科",
    "疼痛科",
    "重症医学科",
    "医学检验科",
    "放射科",
    "超声科",
    "核医学科",
    "病理科",
    "营养科",
    "麻醉科",
]

# 示例提示词
SYMPTOM_EXAMPLES = """
症状-科室映射示例（严格遵循）：
- 胃痛、胃胀、反酸 → 消化内科
- 头痛、头晕、失眠 → 神经内科
- 胸痛、心悸、胸闷 → 心血管内科
- 咳嗽、发热、呼吸困难 → 呼吸内科
- 关节痛、骨折、腰痛 → 骨科
- 皮疹、瘙痒、过敏 → 皮肤科
- 牙痛、牙龈出血 → 口腔科
- 视力下降、眼痛 → 眼科
- 耳朵痛、耳鸣 → 耳鼻喉科
- 腹痛、腹泻、呕吐 → 消化内科
- 女性月经不调、孕期检查 → 妇产科
- 儿童发热、咳嗽 → 儿科
"""


def extract_department(text: str) -> list[str]:
    prompt = f"""
你是一个精准的智能医疗分诊助手。请根据用户提供的症状描述，从预定义的科室列表中严格匹配最相关的科室。

可用科室列表：{', '.join(DEPARTMENTS)}

{SYMPTOM_EXAMPLES}

匹配规则：
1. 每个症状只返回最相关的1-2个科室
2. 严格按照医学常识和上述示例进行映射
3. 如果无法准确匹配，返回"未提及"
4. 只输出科室名称，不包含其他解释

用户症状：{text}

科室：
"""
    try:
        response = client.chat.completions.create(
            model="doubao-1.5-lite-32k",
            messages=[
                {
                    "role": "system",
                    "content": "你是一个严格遵守规则的医疗助手，只返回科室名称。",
                },
                {"role": "user", "content": prompt},
            ],
            temperature=0.1,
            max_tokens=50,
        )
        result_text = response.choices[0].message.content.strip()
        print(f"模型原始人回复: {result_text}")

        if "未提及" in result_text or not result_text:
            return []

        departments = []
        for dept in result_text.split(","):
            dept = dept.strip()
            if dept in DEPARTMENTS:
                departments.append(dept)

        return departments

    except Exception as e:
        print(f"❌ API调用出错: {e}")
        return []


def main():
    parser = argparse.ArgumentParser(description="智能科室推荐系统")
    parser.add_argument("--input", required=True, help="症状描述文本")
    args = parser.parse_args()

    user_input = args.input
    print(f"输入症状：{user_input}")

    departments = extract_department(user_input)

    if departments:
        print(f"✅ 推荐科比: {', '.join(departments)}")
        target_dept = departments[0]

        if target_dept in DEPARTMENT_COORDINATES:
            x, y = DEPARTMENT_COORDINATES[target_dept]
            print(f"hahaha")

            try:
                package_path = get_package_share_directory("send_goal")
                goal_path = "send_goal/goal_pose/goal_pose.txt"
                with open(goal_path, "w") as f:
                    f.write(f"{x} {y}\n")
                os.utime(goal_path, None)  
                print(f"✅ 写入目标坐标: {x}, {y} 到 {goal_path}")

                # 立即读取校验内容
                with open(goal_path, "r") as f_check:
                    content = f_check.read().strip()
                print(f"✅ 目标文件内容校验: {content}")

            except Exception as e:
                print(f"❌ 写入 goal_pose.txt 失败: {e}")
        else:
            print("⚠️ 找不到该科室的坐标，请检查 DEPARTMENT_COORDINATES 配置。")
    else:
        print("⚠️ 未识别到相关科室，建议咨询导诊台")


if __name__ == "__main__":
    main()
