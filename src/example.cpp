#include <core.hpp> 
#include <state_management.hpp>
#include <iostream>
#include <vector>

// Define a delivered class from easysm::State for increase the value of a parameter
class State1 : public easysm::State 
{
    public:
        State1(std::string name) : easysm::State(name) {}

        std::string onExecute() override 
        {
            // Get the parameter "my_param" from the state manager
            auto int_param = easysm::StateManager::state_manager->getParam<int>("my_param");
            *int_param += 1;

            log_warn("running...");

            return "added";
        }
};

// Define a delivered class from easysm::State for checking the value of a parameter
class State2 : public easysm::State 
{
    public:
        State2(std::string name) : easysm::State(name) {}

        std::string onExecute() override 
        {
            auto int_param = easysm::StateManager::state_manager->getParam<int>("my_param");
            log("Current value of 'my_param' -> " + std::to_string(*int_param));

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
class State3 : public easysm::State 
{
    public:
        State3(std::string name) : easysm::State(name) {}

        std::string onExecute() override 
        {
            log("Process complated.");

            return "completed";
        }
};


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "state_manager_example");
    ros::NodeHandle nh;

    // Create or get state manager instance
    auto sm = easysm::StateManager::create<easysm::RosStateManager>(nh, "/execute_feedback");

    // Add states to the state manager
    sm->addState<State1>("State1");
    sm->addState<State2>("State2");
    sm->addState<State3>("State3");

    // Add transition between states to the state manager
    sm->addTransition("Transition4", "added", "State1", "State2");
    sm->addTransition("Transition5", "continue", "State2", "State1");
    sm->addTransition("Transition6", "stop", "State2", "State3");
    
    // Add a parameter to the state manager
    sm->addParam<int>("my_param", 0);
    
    // Execute machine from State1
    sm->executeState("State1");

    ros::spin();
    
    return 0;
}