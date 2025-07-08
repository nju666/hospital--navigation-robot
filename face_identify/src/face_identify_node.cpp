// originbot_face_identify_node.cpp

#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "hobot_cv/hobotcv_imgproc.h"

#include "originbot_face_identify_msgs/msg/face_detection.hpp"

using std::placeholders::_1;
using originbot_face_identify_msgs::msg::FaceDetection;

class OriginbotFaceIdentifyNode : public hobot::dnn_node::DnnNode {
public:
  explicit OriginbotFaceIdentifyNode(const std::string& node_name)
      : hobot::dnn_node::DnnNode(node_name) {
    RCLCPP_INFO(this->get_logger(), "OriginbotFaceIdentifyNode constructor start.");

    this->declare_parameter<std::string>("model_path", "/userdata/dev_ws/src/originbot/originbot_face_identify/model/yolov8n-face_640x640_nv12.bin");
    this->declare_parameter<std::string>("model_name", "yolov8n-face_640x640_nv12");

    if (!GetParams()) {
      RCLCPP_ERROR(this->get_logger(), "GetParams() failed!");
      throw std::runtime_error("Failed to get parameters");
    }

    if (SetNodePara() != 0) {
      RCLCPP_ERROR(this->get_logger(), "SetNodePara() failed!");
      throw std::runtime_error("Failed to set node parameters");
    }

    int init_ret = InitModel();
    if (init_ret != 0) {
      RCLCPP_ERROR(this->get_logger(), "InitModel() failed with code %d", init_ret);
      throw std::runtime_error("Model initialization failed");
    } else {
      RCLCPP_INFO(this->get_logger(), "Model initialized successfully.");
    }

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    face_detection_pub_ = this->create_publisher<FaceDetection>("face_detection", 10);

    subscription_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        "hbmem_img", rclcpp::SensorDataQoS(),
        std::bind(&OriginbotFaceIdentifyNode::ImageCallback, this, _1));
  }

  ~OriginbotFaceIdentifyNode() override = default;

protected:
  int SetNodePara() override {
    if (!dnn_node_para_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "dnn_node_para_ptr_ is nullptr");
      return -1;
    }
    dnn_node_para_ptr_->model_file = model_path_;
    dnn_node_para_ptr_->model_name = model_name_;
    dnn_node_para_ptr_->task_num = 1;
    return 0;
  }

  int InitModel() {
    int ret = this->Init();
    if (ret != 0) {
      RCLCPP_ERROR(this->get_logger(), "Init() returned error code %d", ret);
    }
    return ret;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<FaceDetection>::SharedPtr face_detection_pub_;
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr subscription_;

  std::string model_path_;
  std::string model_name_;

  bool GetParams() {
    try {
      model_path_ = this->get_parameter("model_path").as_string();
      model_name_ = this->get_parameter("model_name").as_string();
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_ERROR(this->get_logger(), "Parameter retrieval failed: %s", e.what());
      return false;
    }
    return true;
  }

  void ImageCallback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg) {
    if (!msg) {
      RCLCPP_WARN(this->get_logger(), "Received empty image message");
      return;
    }

    int width = msg->width;
    int height = msg->height;

    if (msg->data.size() < static_cast<size_t>(width * height * 3 / 2)) {
      RCLCPP_ERROR(this->get_logger(), "Image data size is smaller than expected");
      return;
    }

    std::vector<uint8_t> img_copy(msg->data.begin(), msg->data.end());

    auto pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(img_copy.data()),
        width,
        height,
        640,
        640);
    if (!pyramid) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create NV12 pyramid input");
      return;
    }

    std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>> inputs{pyramid};
    auto rois = std::make_shared<std::vector<hbDNNRoi>>();
    rois->emplace_back(hbDNNRoi{0, 0, 640, 640});

    std::shared_ptr<hobot::dnn_node::DnnNodeOutput> output = std::make_shared<hobot::dnn_node::DnnNodeOutput>();
    int ret = Run(inputs, output, rois, true);
    if (ret != 0) {
      RCLCPP_ERROR(this->get_logger(), "Run DNN failed, ret=%d", ret);
      return;
    }

    if (output->output_tensors.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Output tensor is empty");
      return;
    }

    auto& tensor = output->output_tensors[0];
    if (tensor->sysMem[0].virAddr == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Invalid tensor sysMem");
      return;
    }

    hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    float* data = reinterpret_cast<float*>(tensor->sysMem[0].virAddr);
    if (!data) {
      RCLCPP_ERROR(this->get_logger(), "Output tensor data is nullptr");
      return;
    }

    const int num_detections = 8400;
    const int values_per_detection = 5; // [x, y, w, h, conf]

    FaceDetection face_detection_msg;  // 新建消息，自动字段清空

    int face_count = 0;
    float max_area_ratio = 0.0f;

    for (int i = 0; i < num_detections; ++i) {
      float conf = data[i * values_per_detection + 4];
      if (conf < 0.8f) continue;  // 提高置信度阈值避免假阳性

      float x = data[i * values_per_detection + 0];
      float y = data[i * values_per_detection + 1];
      float w = data[i * values_per_detection + 2];
      float h = data[i * values_per_detection + 3];

      // 简单尺寸过滤
      if (w <= 0 || h <= 0 || w > 640 || h > 640) continue;

      float area_ratio = (w * h) / (640.0f * 640.0f);
      max_area_ratio = std::max(max_area_ratio, area_ratio);

      face_detection_msg.x.push_back(x);
      face_detection_msg.y.push_back(y);
      face_detection_msg.width.push_back(w);
      face_detection_msg.height.push_back(h);
      face_detection_msg.score.push_back(conf);

      face_count++;
    }

    if (face_count > 0) {
      if (max_area_ratio > 0.5f) {
        RCLCPP_INFO(this->get_logger(), "face_identify (max face occupies %.2f%% screen)", max_area_ratio * 100);
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Face(s) detected but none occupy >50%%");
      }
      face_detection_pub_->publish(face_detection_msg);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "No valid faces detected.");
    }

    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = 0.1;
    cmd_msg.angular.z = 0.0;
    publisher_->publish(cmd_msg);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<OriginbotFaceIdentifyNode>("originbot_face_identify_node");
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught in main: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
