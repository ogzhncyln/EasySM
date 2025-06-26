# EasySM
![easysm](https://github.com/user-attachments/assets/d72b760f-e9bd-4885-9c51-770b0b400958)

EasySM is a simple state management library for C++ applications that provides a flexible framework for implementing state machines. It supports both standalone operation and ROS integration.

## Installation

### Standard Installation (without ROS)
1. Clone the repository:
```sh
git clone https://github.com/ogzhncyln/EasySM.git
cd EasySM/easysm_lib
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
cd EasySM/easysm_lib
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

# For ROS support
if(USE_ROS)
    target_compile_definitions(your_executable PRIVATE -DUSE_ROS)
endif()
```

## Library Overview
EasySM implements a state machine architecture with the following key components:
* **States**: Represent different conditions or situations in your application
* **Transitions**: Define how states change from one to another
* **StateManager**: Manages the states, transitions, and parameters using a singleton pattern
* **Parameters**: Shared data that can be accessed and modified by any state

## Basic Usage

### Understanding the State Machine Model
In EasySM, the state machine workflow follows these principles:

1. States are created by inheriting from the `easysm::State` class
2. Transitions connect states and are triggered by specific events
3. When a state executes, it returns an event string
4. The event triggers a transition to the next state
5. The process continues until a state returns an event with no matching transition
6. Parameters can be shared between states through the StateManager

### Creating the State Manager
EasySM uses a singleton pattern for the StateManager. You can create and access it using:

```cpp
// Without ROS
auto sm = easysm::StateManager::create<easysm::DefaultStateManager>(true); // true enables logging

// With ROS
#ifdef USE_ROS
auto sm = easysm::StateManager::create<easysm::RosStateManager>(nh, "/execute_feedback");
#endif
```

The StateManager instance can then be accessed globally via:
```cpp
easysm::StateManager::state_manager
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
        
        // Access shared parameters
        auto param = StateManager::state_manager->getParam<int>("my_param");
        *param += 1;
        
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

// Method 2: Let the state manager create the state for you (Recommended)
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

// Method 2: Let the state manager create the transition for you (Recommended)
sm->addTransition("TransitionName", "next", "State1", "State2");
```

### Using Parameters
Parameters can be shared between states and persist throughout the state machine execution:

```cpp
// Add a parameter
sm->addParam<int>("my_param", 0);
sm->addParam<std::string>("status", "initialized");
sm->addParam<std::vector<int>>("data_list", {1, 2, 3});

// In a state, retrieve and modify the parameter
auto param = easysm::StateManager::state_manager->getParam<int>("my_param");
*param += 1;

// Remove the parameter when no longer needed
sm->removeParam("my_param");
```

### Complete Example
Here's a complete example demonstrating EasySM:

```cpp
#include <easysm/core.hpp> 
#include <easysm/state_management.hpp>
#include <iostream>
#include <vector>

#ifdef USE_ROS
#include <ros/ros.h>
#endif

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

#ifdef USE_ROS
            ros::Duration(1.0).sleep();
#endif

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

#ifdef USE_ROS
            ros::Duration(1.0).sleep();
#endif

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
#ifdef USE_ROS
    ros::init(argc, argv, "state_manager_example");
    ros::NodeHandle nh;

    // Create state manager instance with ROS
    auto sm = StateManager::create<RosStateManager>(nh, "/monitor_cmd");
#else
    // Create state manager instance without ROS
    auto sm = StateManager::create<DefaultStateManager>(true);
#endif

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

#ifdef USE_ROS
    ros::spin();
#endif
    
    return 0;
}
```

The execution flow of this example is:
* State1 increments my_param and returns "added"
* "added" triggers Transition1 to State2
* State2 checks my_param value:
  * If <= 5: returns "continue", triggering Transition2 back to State1
  * If > 5: returns "stop", triggering Transition3 to State3
* State3 outputs completion message and returns "completed"
* No transition is defined for "completed", so the state machine terminates

### Example Diagram
![easysm_diagram](https://github.com/user-attachments/assets/9b943a21-ecab-4288-b62f-d733bd4c0f6f)

## Advanced Usage

### Logging in States
EasySM provides built-in logging methods in the State class:

```cpp
void log(std::string message);     // Normal log message
void log_err(std::string message);  // Error log message
void log_warn(std::string message); // Warning log message
```

These methods will be handled by the state manager's `logFeedback` method, which will output appropriately based on the manager type:
- **DefaultStateManager**: Outputs to console (if logging is enabled)
- **RosStateManager**: Publishes log messages to ROS topic

### Parameter Management
The StateManager provides comprehensive parameter management:

```cpp
// Adding parameters of different types
sm->addParam<int>("counter", 0);
sm->addParam<std::string>("status", "ready");
sm->addParam<std::vector<int>>("data", {1, 2, 3});
sm->addParam<double>("threshold", 3.14);

// Accessing parameters from any state
auto counter = StateManager::state_manager->getParam<int>("counter");
auto status = StateManager::state_manager->getParam<std::string>("status");

// Modifying parameters
*counter = 10;
*status = "running";

// Removing parameters
sm->removeParam("counter");
```

### Custom State Managers
You can create custom state managers by inheriting from `easysm::StateManager` and implementing the required virtual methods:

```cpp
class MyStateManager : public easysm::StateManager
{
public:
    void executionFeedback(std::string executed_object_name) override {
        // Custom feedback implementation
        std::cout << "Executed: " << executed_object_name << std::endl;
    }
    
    void executionLoopTerminated() override {
        // Custom termination handling
        std::cout << "State machine terminated" << std::endl;
    }
    
    void logFeedback(std::string state_name, std::string log_type, std::string data) override {
        // Custom log handling
        std::cout << "[" << state_name << "][" << log_type << "] " << data << std::endl;
    }
};
```

### ROS Integration
When built with ROS support (`-DUSE_ROS=ON`), EasySM provides:

- **RosStateManager**: Publishes state execution feedback and logs to ROS topics
- **ROS-aware timing**: Use `ros::Duration().sleep()` in states
- **Topic-based monitoring**: Monitor state machine execution via ROS topics

```cpp
#ifdef USE_ROS
// Create ROS state manager
auto sm = StateManager::create<RosStateManager>(nh, "/state_feedback");

// The state manager will publish:
// - "executed:<state_name>" when a state executes
// - "terminated" when state machine ends
// - "logging:<state_name>:<log_type>:<message>" for log messages
#endif
```

### State Machine Serialization

EasySM provides functionality to save the current state machine structure to a file for debugging, visualization, or backup purposes:

```cpp
// Save the state machine structure to a file
sm->saveTree("/path/to/directory/", "tree_filename");
```

**Parameters:**
- `directory_path`: The directory where the tree file will be saved
- `filename`: The name of the file (without extension)

**Usage Example:**
```cpp
// After setting up your state machine
sm->addState<State1>("State1");
sm->addState<State2>("State2");
sm->addState<State3>("State3");

sm->addTransition("Transition1", "added", "State1", "State2");
sm->addTransition("Transition2", "continue", "State2", "State1");
sm->addTransition("Transition3", "stop", "State2", "State3");

// Save the state machine structure
sm->saveTree("/home/user/logs/", "my_state_machine");
```

This feature is useful for:
- **Debugging**: Visualize the state machine structure
- **Documentation**: Generate diagrams of your state machine
- **Analysis**: Review state transitions and connections
- **Backup**: Save state machine configurations

The saved file contains information about all states, transitions, and their relationships, making it easy to understand and debug complex state machines.

## API Reference

### StateManager Methods
- `addState<T>(std::string name)` - Add a state of type T
- `addState(std::shared_ptr<State> state)` - Add a pre-created state
- `addTransition(std::string name, std::string trigger, std::string source, std::string target)` - Add a transition
- `addParam<T>(std::string name, T value)` - Add a parameter
- `getParam<T>(std::string name)` - Get a parameter
- `removeParam(std::string name)` - Remove a parameter
- `executeState(std::string name)` - Start execution from a specific state

### State Methods
- `onExecute(std::shared_ptr<Transition> transition)` - Pure virtual method to implement
- `log(std::string message)` - Log normal message
- `log_warn(std::string message)` - Log warning message
- `log_err(std::string message)` - Log error message
- `getName()` - Get state name

## License
This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE) file for details.

