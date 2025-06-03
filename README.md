# EasySM
EasySM is a simple state management library for C++ applications that provides a flexible framework for implementing state machines. It supports both standalone operation and ROS (Robot Operating System) integration.

# Installation

## Standard Installation (without ROS)
1. Clone the repository:
<pre>git clone https://github.com/ogzhncyln/EasySM.git
cd EasySM</pre>
    
2. Create build directory and build the library:
<pre>mkdir build && cd build
cmake ..
make</pre>

3. Install the library:
<pre>sudo make install</pre>

## Installation with ROS Support
1. Clone the repository:
<pre>git clone https://github.com/ogzhncyln/EasySM.git
cd EasySM</pre>
    
2. Create build directory and build the library:
<pre>mkdir build && cd build
cmake .. -DUSE_ROS=ON
make</pre>

3. Install the library:
<pre>sudo make install</pre>

## Using EasySM in Your CMake Project
<pre>find_package(easysm REQUIRED)
add_executable(your_executable main.cpp)
target_link_libraries(your_executable easysm::easysm)</pre>

# Library Overview
EasySM implements a state machine architecture with the following key components:
* **States**: Represent different conditions or situations in your application
* **Transitions**: Define how states change from one to another
* **StateManager**: Manages the states and transitions

# Basic Usage
Understanding the State Machine Model
In EasySM, the state machine workflow follows these principles:

1. States are created by inheriting from the `easysm::State` class
2. Transitions connect states and are triggered by specific events
3. When a state executes, it returns an event string
4. The event triggers a transition to the next state
5. The process continues until a state returns an event with no matching transition

### Creating Custom States
States are created by inheriting from the `easysm::State` class and implementing the `onExecute()` method:
<pre>class MyState : public easysm::State 
{
public:
    MyState(std::string state_name) : easysm::State(state_name) {}
    
    std::string onExecute() override {
        // Your state logic here
        
        // Return an event string that will trigger a transition
        return "some_event";
    }
};</pre>

### Setting Up Transitions
Transitions connect states and are triggered by specific event strings:
<pre>auto stateA = std::make_shared<StateA>();
auto stateB = std::make_shared<StateB>();

// Create a transition from stateA to stateB that triggers on "next" event
auto transition = std::make_shared<easysm::Transition>(
    "TransitionName",  // Transition name
    "next",           // Trigger event
    stateA,           // Source state
    stateB            // Target state
);</pre>

### Using Parameters
Parameters can be shared between states:
<pre>// Add a parameter
sm->addParam<int>("my_param", 0);

// In a state, retrieve and modify the parameter
auto param = state_manager->getParam<int>("my_param");</pre>

### Complete Example Explanation
Let's analyze the provided example in `example.cpp`:

<pre>int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "state_manager_example");
    ros::NodeHandle nh;

    // Create a ROS-enabled state manager
    auto sm = std::make_shared<easysm::RosStateManager>(nh,"/execute_feedback");
    
    // Create three states
    auto state1 = std::make_shared<State1>();
    auto state2 = std::make_shared<State2>();
    auto state3 = std::make_shared<State3>();
    
    // Create transitions between states
    auto transition1 = std::make_shared<easysm::Transition>("Transition1", "added", state1, state2);
    auto transition2 = std::make_shared<easysm::Transition>("Transition2", "continue", state2, state1);
    auto transition3 = std::make_shared<easysm::Transition>("Transition3", "stop", state2, state3);
    
    // Add a parameter that will be shared between states
    sm->addParam<int>("my_param", 0);
    
    // Register states with the manager
    sm->addState(state1);
    sm->addState(state2);
    sm->addState(state3);
    
    // Register transitions with the manager
    sm->addTransition(transition1);
    sm->addTransition(transition2);
    sm->addTransition(transition3);
    
    // Start the state machine at State1
    sm->executeState("State1");

    // Keep the ROS node running
    ros::spin();
    
    return 0;
}</pre>

This example creates a state machine with three states:
1. State1: Increments a parameter and transitions to State2
2. State2: Checks the parameter value and either transitions back to State1 or to State3
3. State3: Outputs completion message
   
The execution flow is:
* Start at State1
* State1 increments my_param and returns "added"
* "added" triggers Transition1 to State2
* State2 checks my_param value:
* If <= 5: returns "continue", triggering Transition2 back to State1
* If > 5: returns "stop", triggering Transition3 to State3
* State3 outputs completion message and returns "completed"
* No transition is defined for "completed", so the state machine terminates

# Using Without ROS
If you don't need ROS integration, you can use the DefaultStateManager:

The rest of the code remains the same.

``` auto sm = std::make_shared<easysm::DefaultStateManager>(true);   // true enables logging ```

# Advanced Usage
Custom State Managers
You can create custom state managers by inheriting from `easysm::StateManager` and implementing the required virtual methods:

<pre>class MyStateManager : public easysm::StateManager
{
public:
    void executeFeedback(std::string executed_object_name) override {
        // Custom feedback implementation
    }
    
    void executionLoopTerminated() override {
        // Custom termination handling
    }
};</pre>

