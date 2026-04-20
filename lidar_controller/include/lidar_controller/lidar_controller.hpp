#ifndef LIDAR_CONTROLLER__LIDAR_CONTROLLER_HPP_
#define LIDAR_CONTROLLER__LIDAR_CONTROLLER_HPP_

#include <atomic>
#include <thread>
#include <map>
#include <algorithm>
#include <iterator>
#include <queue>
#include <mutex>
#include <condition_variable>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/msg/image.hpp>
// #include "controller_msg/msg/custom_point.hpp"
// #include "controller_msg/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <sensor_base/data_layouts.hpp>
#include <sensor_base/spsc_queue.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lidar_controller/lidar_controller_parameters.hpp>

namespace lidar_controller {

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;


struct LidarPublishTask {
    uint64_t timestamp_nanos{0};
    uint64_t update_count;
    // one of lidar data
    CustomMsg msg; // only lidar data
    PointCloud2 point_cloud_msg; // point cloud data
    sensor_base::LidarDataLayout raw_data;
};

struct LidarStreamContext {
    std::string interface_name;
    std::string topic_name;
    std::string frame_id;

    uint64_t last_update_count{0};
    double last_ptr_value{0.0};

    std::mutex mutex_;
    std::condition_variable cv_;

    // for dynamic extrinsics
    bool enable_dynamic_extrinsics{false};
    Eigen::Affine3f current_extrinsics{Eigen::Affine3f::Identity()};
    std::mutex extrinsics_mutex_;
    bool extrinsics_updated_{false};

    // for worker thread
    std::thread thread_;
    std::vector<LidarPublishTask> publish_tasks_pool_;
    sensor_base::SPSCQueue<int, 8> free_queue_;
    sensor_base::SPSCQueue<int, 8> work_queue_;

    rclcpp::Publisher<CustomMsg>::SharedPtr lidar_livox_pub_ = nullptr;
    rclcpp::Publisher<PointCloud2>::SharedPtr lidar_point_cloud_pub_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dynamic_livox_sub_ = nullptr;
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LidarController : public controller_interface::ControllerInterface {

    private:
        
    public:
        LidarController();

        CallbackReturn on_init() override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;
    
    public:
            
        void publish_worker(std::shared_ptr<LidarStreamContext> ctx);
        void FillLidarPublishTaskWithPoints(LidarPublishTask& task, const sensor_base::LidarDataLayout& data, bool do_transform, const Eigen::Affine3f& transform, std::shared_ptr<LidarStreamContext> ctx);
        void FillLidarPublishTaskWithPoints2(LidarPublishTask& task, const sensor_base::LidarDataLayout& data, bool do_transform, const Eigen::Affine3f& transform, std::shared_ptr<LidarStreamContext> ctx);
        void FillBothMessagesSinglePass(LidarPublishTask& task, const sensor_base::LidarDataLayout& data, bool do_transform, const Eigen::Affine3f& transform, std::shared_ptr<LidarStreamContext> ctx);

    protected:
        

        std::vector<std::shared_ptr<LidarStreamContext>> lidar_stream_contexts_;
        
        // params getter
        std::shared_ptr<lidar_controller::ParamListener> param_listener_;
        lidar_controller::Params params_;

        // about thread
        std::vector<std::thread> threads_;
        std::atomic<bool> is_running_{false};
};

} // namespace image_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    lidar_controller::LidarController, controller_interface::ControllerInterface)

#endif // LIDAR_CONTROLLER__LIDAR_CONTROLLER_HPP_
