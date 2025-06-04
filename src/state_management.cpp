#include <state_management.hpp>

namespace easysm
{
    DefaultStateManager::DefaultStateManager(bool logging) : logging(logging) {
    }
    
    DefaultStateManager::~DefaultStateManager() {
    }
    
    void DefaultStateManager::executionFeedback(std::string executed_object_name) {
        if (logging) {
            std::cout << "Executed: " << executed_object_name << std::endl;
        }
    }
    
    void DefaultStateManager::executionLoopTerminated() {
        if (logging) {
            std::cout << "Execution loop terminated" << std::endl;
        }
    }
    
#ifdef USE_ROS
    RosStateManager::RosStateManager(ros::NodeHandle& nh, std::string topic_name) {
        feedback_publisher = nh.advertise<std_msgs::String>(topic_name, 10);
        ros::Duration(0.1).sleep();
    }
    
    RosStateManager::~RosStateManager() {
    }
    
    void RosStateManager::executionFeedback(std::string executed_object_name) {
        std_msgs::String msg;
        msg.data = "executed:" + executed_object_name;
        feedback_publisher.publish(msg);

    }
    
    void RosStateManager::executionLoopTerminated() {
        std_msgs::String msg;
        msg.data = "terminated";
        feedback_publisher.publish(msg);
    }
#endif
}