# EasySM
![easysm](https://github.com/user-attachments/assets/d72b760f-e9bd-4885-9c51-770b0b400958)

EasySM is a simple state management library for C++ applications that provides a flexible framework for implementing state machines. It supports both standalone operation and ROS integration.

## Installation

### Standard Installation (without ROS)
1. Clone the repository:
```sh
git clone https://github.com/ogzhncyln/EasySM.git
cd EasySM
```
    
2. Create build directory and build the library:
```sh
mkdir build && cd build
cmake ..
make
```

3. Install the library:
```sh
sudo make install
```

### Installation with ROS Support
1. Clone the repository:
```sh
git clone https://github.com/ogzhncyln/EasySM.git
cd EasySM
```
    
2. Create build directory and build the library:
```sh
mkdir build && cd build
cmake .. -DUSE_ROS=ON
make
```

3. Install the library:
```sh
sudo make install
```

### Using EasySM in Your CMake Project
```cmake
find_package(easysm REQUIRED)
add_executable(your_executable main.cpp)
target_link_libraries(your_executable easysm::easysm)
```

## Library Overview
EasySM implements a state machine architecture with the following key components:
* **States**: Represent different conditions or situations in your application
* **Transitions**: Define how states change from one to another
* **StateManager**: Manages the states and transitions

## Basic Usage
### Understanding the State Machine Model
In EasySM, the state machine workflow follows these principles:

1. States are created by inheriting from the `easysm::State` class.
2. Transitions connect states and are triggered by specific events.
3. When a state executes, it returns an event string.
4. The event triggers a transition to the next state.
5. The process continues until a state returns an event with no matching transition.

### Creating the State Manager
EasySM provides a static creation method to easily create and access your state manager:

```cpp
// Without ROS
auto sm = easysm::StateManager::create<easysm::DefaultStateManager>(true); // true enables logging

// With ROS
auto sm = easysm::StateManager::create<easysm::RosStateManager>(nh, "/execute_feedback");
```

### Creating Custom States
States are created by inheriting from the `easysm::State` class and implementing the `onExecute()` method:

```cpp
class MyState : public easysm::State 
{
public:
    MyState(std::string state_name) : easysm::State(state_name) {}
    
    std::string onExecute(std::shared_ptr<Transition> transition) override {
        // Check if state was called by a transition
        if(transition) {
            std::cout << "Transition that called this state: " << transition->getName() << std::endl;
        }
        
        // Your state logic here
        
        // Log information if needed
        log("This is a normal log message");
        log_warn("This is a warning message");
        log_err("This is an error message");
        
        // Return an event string that will trigger a transition
        return "some_event";
    }
};
```

### Adding States to the State Manager
You can add states to the state manager in two ways:

```cpp
// Method 1: Create the state manually and add it
auto state1 = std::make_shared<MyState>("State1");
sm->addState(state1);

// Method 2: Let the state manager create the state for you
sm->addState<MyState>("State1");
```

### Setting Up Transitions
Transitions connect states and are triggered by specific event strings:

```cpp
// Method 1: Create the transition manually and add it
auto transition = std::make_shared<easysm::Transition>(
    "TransitionName",  // Transition name
    "next",           // Trigger event
    state1,           // Source state
    state2            // Target state
);
sm->addTransition(transition);

// Method 2: Let the state manager create the transition for you
sm->addTransition("TransitionName", "next", "State1", "State2");
```

### Using Parameters
Parameters can be shared between states:

```cpp
// Add a parameter
sm->addParam<int>("my_param", 0);

// In a state, retrieve and modify the parameter
auto param = easysm::StateManager::state_manager->getParam<int>("my_param");
*param += 1;

// Remove the parameter
sm->removeParam("my_param");
```

### Saving the State Tree
EasySM allows saving the state machine structure to a file for visualization or debugging:

```cpp
sm->saveTree("/path/to/directory", "state_machine");
```

### Complete Example
Here's a complete example demonstrating EasySM:

```cpp
#include <easysm/core.hpp> 
#include <easysm/state_management.hpp>
#include <iostream>
#include <vector>
#include <ros/ros.h>

using namespace easysm;

// Define a state that increments a parameter
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

// Define a state that checks the parameter value
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

// Define a state for completing the process
class State3 : public State 
{
    public:
        State3(std::string name) : State(name) {}

        std::string onExecute(std::shared_ptr<Transition> transition) override 
        {
            if(transition)
                std::cout << "State3 executor transition -> " << transition->getName() << std::endl;

            log("Process completed.");

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

    // Add transitions between states to the state manager
    sm->addTransition("Transition1", "added", "State1", "State2");
    sm->addTransition("Transition2", "continue", "State2", "State1");
    sm->addTransition("Transition3", "stop", "State2", "State3");
    
    // Add a parameter to the state manager
    sm->addParam<int>("my_param", 0);

    // Execute machine from State1
    sm->executeState("State1");

    ros::spin();
    
    return 0;
}
```

#### Explanation of the Example
This example demonstrates the use of EasySM with ROS integration. It includes three states (`State1`, `State2`, and `State3`) and transitions between them:

1. **State1**:
   - Increments the parameter `my_param`.
   - Waits for 1 second using ROS.
   - Returns the event `"added"`.

2. **State2**:
   - Checks the value of `my_param`.
   - Logs the current value of `my_param`.
   - Waits for 1 second using ROS.
   - Returns `"continue"` if `my_param <= 5`, or `"stop"` if `my_param > 5`.

3. **State3**:
   - Logs a completion message.
   - Returns `"completed"`.

#### Execution Flow
- The state machine starts with `State1`.
- `State1` increments `my_param` and triggers the `"added"` event, transitioning to `State2`.
- `State2` checks the value of `my_param`:
  - If `my_param <= 5`, it triggers the `"continue"` event, transitioning back to `State1`.
  - If `my_param > 5`, it triggers the `"stop"` event, transitioning to `State3`.
- `State3` logs a completion message and terminates the state machine.

This example showcases how to use EasySM for state management with ROS, including parameter handling, transitions, and logging.### Complete Example
Here's a complete example demonstrating EasySM:

```cpp
#include <easysm/core.hpp> 
#include <easysm/state_management.hpp>
#include <iostream>
#include <vector>
#include <ros/ros.h>

using namespace easysm;

// Define a state that increments a parameter
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

// Define a state that checks the parameter value
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

// Define a state for completing the process
class State3 : public State 
{
    public:
        State3(std::string name) : State(name) {}

        std::string onExecute(std::shared_ptr<Transition> transition) override 
        {
            if(transition)
                std::cout << "State3 executor transition -> " << transition->getName() << std::endl;

            log("Process completed.");

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

    // Add transitions between states to the state manager
    sm->addTransition("Transition1", "added", "State1", "State2");
    sm->addTransition("Transition2", "continue", "State2", "State1");
    sm->addTransition("Transition3", "stop", "State2", "State3");
    
    // Add a parameter to the state manager
    sm->addParam<int>("my_param", 0);

    // Execute machine from State1
    sm->executeState("State1");

    ros::spin();
    
    return 0;
}
```

#### Explanation of the Example
This example demonstrates the use of EasySM with ROS integration. It includes three states (`State1`, `State2`, and `State3`) and transitions between them:

1. **State1**:
   - Increments the parameter `my_param`.
   - Waits for 1 second using ROS.
   - Returns the event `"added"`.

2. **State2**:
   - Checks the value of `my_param`.
   - Logs the current value of `my_param`.
   - Waits for 1 second using ROS.
   - Returns `"continue"` if `my_param <= 5`, or `"stop"` if `my_param > 5`.

3. **State3**:
   - Logs a completion message.
   - Returns `"completed"`.

#### Execution Flow
- The state machine starts with `State1`.
- `State1` increments `my_param` and triggers the `"added"` event, transitioning to `State2`.
- `State2` checks the value of `my_param`:
  - If `my_param <= 5`, it triggers the `"continue"` event, transitioning back to `State1`.
  - If `my_param > 5`, it triggers the `"stop"` event, transitioning to `State3`.
- `State3` logs a completion message and terminates the state machine.

This example showcases how to use EasySM for state management with ROS, including parameter handling, transitions, and logging.

