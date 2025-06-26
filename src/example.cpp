#include <easysm/core.hpp> 
#include <easysm/state_management.hpp>
#include <iostream>
#include <vector>
#include <ros/ros.h>

using namespace easysm;

// Define a delivered class from easysm::State for increase the value of a parameter
class State1 : public State 
{
    public:
        State1(std::string name) : State(name) {}

        std::string onExecute(std::shared_ptr<Transition> transition) override 
        {
            if(transition)
                std::cout << "State1 executor transition -> " << transition->getName() << std::endl;

            // Get the parameter "my_param" from the state manager
            auto int_param = StateManager::state_manager->getParam<int>("my_param");
            *int_param += 1;

            // Wait for 1 second using ROS
            ros::Duration(1.0).sleep();

            log_warn("running...");

            return "added";
        }
};

// Define a delivered class from easysm::State for checking the value of a parameter
class State2 : public State 
{
    public:
        State2(std::string name) : State(name) {}

        std::string onExecute(std::shared_ptr<Transition> transition) override 
        {
            if(transition)
                std::cout << "State2 executor transition -> " << transition->getName() << std::endl;

            auto int_param = StateManager::state_manager->getParam<int>("my_param");
            log("Current value of 'my_param' -> " + std::to_string(*int_param));
            ros::Duration(1.0).sleep();

            if(*int_param > 5) 
            {
                return "stop";
            }
            else
            {
                return "continue";
            }
        }
};

// Define a delivered class from easysm::State for completing the process
class State3 : public State 
{
    public:
        State3(std::string name) : State(name) {}

        std::string onExecute(std::shared_ptr<Transition> transition) override 
        {
            if(transition)
                std::cout << "State3 executor transition -> " << transition->getName() << std::endl;

            log("Process complated.");

            return "completed";
        }
};


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "state_manager_example");
    ros::NodeHandle nh;

    // Create or get state manager instance
    auto sm = StateManager::create<RosStateManager>(nh, "/monitor_cmd");

    // Add states to the state manager
    sm->addState<State1>("State1");
    sm->addState<State2>("State2");
    sm->addState<State3>("State3");

    // Add transition between states to the state manager
    sm->addTransition("Transition1", "added", "State1", "State2");
    sm->addTransition("Transition2", "continue", "State2", "State1");
    sm->addTransition("Transition3", "stop", "State2", "State3");
    
    // Add a parameter to the state manager
    sm->addParam<int>("my_param", 0);

    //sm->saveTree("/home/usr/", "example_tree");
    
    // Execute machine from State1
    sm->executeState("State1");

    ros::spin();
    
    return 0;
}