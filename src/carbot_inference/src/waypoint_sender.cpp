#include "carbot_inference/waypoint_sender.hpp"

#include <rclcpp/rclcpp.hpp>
#include "carbot_inference/msg/waypoints.hpp"
#include <InferPipelineManager.hpp>

#include <iostream>


WaypointSender::WaypointSender() : Node("waypoint_sender") {
    this->declare_parameter("debug_mode", false);
    this->declare_parameter("debug_output_dir", "");
    this->declare_parameter("num_waypoints", 6);
    this->declare_parameter("model_name", "simple_traj");
    this->declare_parameter("is_discretise", true);
    auto num_waypoints = this->get_parameter("num_waypoints").as_int();
    auto debug_output_dir = this->get_parameter("debug_output_dir").as_string();
    auto debug_mode = this->get_parameter("debug_mode").as_bool();
    auto model_name = this->get_parameter("model_name").as_string();
    is_discretise_ = this->get_parameter("is_discretise").as_bool();

    std::string save_path = "";
    if (debug_mode) {
        save_path = debug_output_dir;
    }

    int num_bins = 19;

    model_context_ = ModelContext(num_waypoints, 1, num_bins);
    std::cout << model_context_.INPUT_SIZE << std::endl;


    if (is_discretise_) {
    undiscretise_predict_ = std::make_unique<UndiscretisePredict>(model_context_.NUM_WAYPOINTS, model_context_.NUM_BINS);
    }

    infer_pipeline_manager_ = std::make_unique<InferPipelineManager>(
        model_context_,
        save_path,
        "/home/tom/carbot_inference/config/config_preprocess.txt", 
        "/home/tom/models/carbot_v1/" + model_name + ".engine");

    if (!infer_pipeline_manager_->setup()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set up InferPipelineManager.");
        throw std::runtime_error("InferPipelineManager setup failed");
    }

    if (!infer_pipeline_manager_->startPipelineAsync()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start inference pipeline.");
        throw std::runtime_error("InferPipelineManager startPipelineAsync failed");
    }

    publisher_ = this->create_publisher<carbot_inference::msg::Waypoints>("waypoints", 10);
    publish_thread_ = std::thread(&WaypointSender::publish_waypoints, this);

    rclcpp::on_shutdown([this]() {
        infer_pipeline_manager_->stopPipeline();
        if (publish_thread_.joinable()) {
            publish_thread_.join();
        }
    });
}

void WaypointSender::publish_waypoints() {
    std::vector<float> outputs;
    while (infer_pipeline_manager_->getLatestResult(outputs) && rclcpp::ok()) {
        auto waypoints_msg = carbot_inference::msg::Waypoints();

        if (is_discretise_) {
            std::vector<float> discretise_outputs = undiscretise_predict_->undiscretiseNSample(outputs);
            waypoints_msg.waypoints = discretise_outputs;
        } else {
            waypoints_msg.waypoints = outputs;
        }
        publisher_->publish(waypoints_msg);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
