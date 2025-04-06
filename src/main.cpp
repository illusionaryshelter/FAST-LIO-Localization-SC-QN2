#include "fast_lio_localization_sc_qn.hpp"

#include <kindr/minimal/quat-transformation.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
int main(int argc, char **argv) {

   rclcpp::init(argc, argv);
   auto node = std::make_shared<FastLioLocalizationScQn>();
   rclcpp::spin(node->get_node_base_interface());
   rclcpp::shutdown();
   return 0;
}
