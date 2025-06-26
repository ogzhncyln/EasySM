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

1. States are created by inheriting from the `easysm::State` class
2. Transitions connect states and are triggered by specific events
3. When a state executes, it returns an event string
4. The event triggers a transition to the next state
5. The process continues until a state returns an event with no matching transition

### Creating the State Manager
EasySM provides a static creation method to easily create and access your state manager:

```cpp
// Without ROS
auto sm = easysm::StateManager::create<easysm::DefaultStateManager>(true); // true enables logging

// With ROS
auto sm = easysm::StateManager::create<easysm::RosStateManager>(nh, "/execute_feedback");
```

### Creating Custom States
States are created by inheriting from the `easysm::State` class and implementing the [`onExecute()`](/home/oguzhan/crab_ws/src/easysm_test/src/main.cpp ) method:

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

### Complete Example
Here's a complete example demonstrating EasySM:

The example code is located in the [`example.cpp`](src/example.cpp) file. It showcases the usage of EasySM with ROS integration. Below is an overview of the example:

#### Workflow
1. **State1**: Increments a parameter `my_param` and returns the event `"added"`.
2. **State2**: Checks the value of `my_param`:
   - If `my_param <= 5`: Returns `"continue"`, triggering a transition back to **State1**.
   - If `my_param > 5`: Returns `"stop"`, triggering a transition to **State3**.
3. **State3**: Outputs a completion message and returns `"completed"`. Since no transition is defined for `"completed"`, the state machine terminates.

#### Code Highlights
- **State1**: Implements logic to increment a parameter and log warnings.
- **State2**: Implements logic to check the parameter value and log its state.
- **State3**: Implements logic to log a completion message.
- **Transitions**: Connect states based on event triggers.
- **ROS Integration**: Uses `ros::Duration` for delays and publishes feedback to a ROS topic.

#### Execution Flow
The state machine starts from **State1** and transitions between states based on the parameter value and event triggers.

For the full implementation, refer to the [`example.cpp`](src/example.cpp) file.

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

These methods will be handled by the state manager's logFeedback method, which will output appropriately based on the manager type.

### Custom State Managers
You can create custom state managers by inheriting from `easysm::StateManager` and implementing the required virtual methods:

```cpp
class MyStateManager : public easysm::StateManager
{
public:
    void executionFeedback(std::string executed_object_name) override {
        // Custom feedback implementation
    }

    void executionLoopBegin() override {
        // Custom termination handling
    }
    
    void executionLoopTerminated() override {
        // Custom termination handling
    }
    
    void logFeedback(std::string state_name, std::string log_type, std::string data) override {
        // Custom log handling
    }
};
```

