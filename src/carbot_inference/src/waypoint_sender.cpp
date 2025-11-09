#include "carbot_inference/waypoint_sender.hpp"


#include <rclcpp/rclcpp.hpp>
#include "carbot_inference/msg/waypoints.hpp"
#include <InferPipelineManager.hpp>


WaypointSender::WaypointSender() : Node("waypoint_sender") {

    infer_pipeline_manager_ = std::make_unique<InferPipelineManager>();

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
    std::vector<float> waypoints;
    while (infer_pipeline_manager_->getLatestResult(waypoints) && rclcpp::ok()) {
        auto waypoints_msg = carbot_inference::msg::Waypoints();
        waypoints_msg.waypoints = waypoints;
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