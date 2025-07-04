#pragma once

#include <iostream>
#include <easysm/core.hpp>

#ifdef USE_ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#endif

namespace easysm
{
    class DefaultStateManager : public StateManager
    {
        public:
            DefaultStateManager(bool logging = false);
            ~DefaultStateManager();

            void executionFeedback(std::string executed_object_name) override;
            void executionLoopBegin() override;
            void executionLoopTerminated() override;
            void logFeedback(std::string state_name, std::string log_type, std::string data) override;

        private:
            bool logging = false;
    };

#ifdef USE_ROS
    class RosStateManager : public StateManager
    {
        public:
            RosStateManager(ros::NodeHandle& nh,std::string topic_name);
            ~RosStateManager();

            void executionFeedback(std::string executed_object_name) override;
            void executionLoopBegin() override;
            void executionLoopTerminated() override;
            void logFeedback(std::string state_name, std::string log_type, std::string data) override;

        private:
            ros::Publisher feedback_publisher;
            
    };
#endif

    
}