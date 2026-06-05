#pragma once
#include <rclcpp/rclcpp.hpp>
#include "carbot_inference/msg/waypoints.hpp"
#include <InferPipelineManager.hpp>
#include <Coms.hpp>
#include "UndiscretisePredict.hpp"

#include <vector>
#include <memory>
#include <thread>

class WaypointSender : public rclcpp::Node {
public:
  WaypointSender();
private:
    void publish_waypoints();
    rclcpp::Publisher<carbot_inference::msg::Waypoints>::SharedPtr publisher_;

    std::unique_ptr<InferPipelineManager> infer_pipeline_manager_;
    std::thread publish_thread_;
    ModelContext model_context_;

    bool is_discretise_ = false;
    std::unique_ptr<UndiscretisePredict> undiscretise_predict_;

};
