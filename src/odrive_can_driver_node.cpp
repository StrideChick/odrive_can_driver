#include "rclcpp/rclcpp.hpp"
#include "odrive_can_driver/odrive_can_driver.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);    
    rclcpp::NodeOptions options;
    auto node = std::make_shared<odrive_can_driver::ODriveCANDriver>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
