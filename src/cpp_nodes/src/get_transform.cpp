#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// using namespace std::chrono_literals;

class DepthToMapTransformPublisher : public rclcpp::Node
{
public:
    DepthToMapTransformPublisher()
        : Node("depth_to_map_transform_publisher"), tf_buffer_(this->get_clock())
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("depth_to_map_transform", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DepthToMapTransformPublisher::publishTransform, this));
    }

private:
    void publishTransform()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf_buffer_.lookupTransform("odom", "depth_camera_link", tf2::TimePointZero);
            
            printf("Transform: %f %f %f\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);

            publisher_->publish(transformStamped);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "TransformException: %s", ex.what());
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToMapTransformPublisher>());
    rclcpp::shutdown();
    return 0;
}
