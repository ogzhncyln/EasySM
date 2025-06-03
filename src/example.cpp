#include <core.hpp> 
#include <state_management.hpp>
#include <iostream>
#include <vector>

class State1 : public easysm::State 
{
    public:
        State1() 
            : easysm::State("State1") {}

        std::string onExecute() override {
            auto int_param = state_manager->getParam<int>("my_param");
            *int_param += 1;

            return "added";
        }
};

class State2 : public easysm::State 
{
    public:
        State2() 
            : easysm::State("State2") {}

        std::string onExecute() override {
            auto int_param = state_manager->getParam<int>("my_param");
            std::cout << "Integer parameter: " << *int_param << std::endl;

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

class State3 : public easysm::State 
{
    public:
        State3() 
            : easysm::State("State3") {}

        std::string onExecute() override {
            std::cout << "Process completed." << std::endl;

            return "completed";
        }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "state_manager_example");
    ros::NodeHandle nh;

    auto sm = std::make_shared<easysm::RosStateManager>(nh,"/execute_feedback");
    
    auto state1 = std::make_shared<State1>();
    auto state2 = std::make_shared<State2>();
    auto state3 = std::make_shared<State3>();
    
    auto transition1 = std::make_shared<easysm::Transition>("Transition1", "added", state1, state2);
    auto transition2 = std::make_shared<easysm::Transition>("Transition2", "continue", state2, state1);
    auto transition3 = std::make_shared<easysm::Transition>("Transition3", "stop", state2, state3);
    
    sm->addParam<int>("my_param", 0);
    
    sm->addState(state1);
    sm->addState(state2);
    sm->addState(state3);
    
    sm->addTransition(transition1);
    sm->addTransition(transition2);
    sm->addTransition(transition3);
    
    sm->executeState("State1");

    ros::spin();
    
    return 0;
}