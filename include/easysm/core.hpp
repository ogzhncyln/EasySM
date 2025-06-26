#pragma once

#include <iostream>
#include <memory>
#include <vector>

namespace easysm
{
    class State;
    class Transition;

    class StateManager 
    {
        public:
            StateManager();
            ~StateManager();

            void addState(std::shared_ptr<State> state); // Adds a state to the manager

            template <typename T, typename... Args>
            void addState(Args&&... args)
            {
                static_assert(std::is_base_of<State, T>::value, "T must derive from State");

                auto instance = std::make_shared<T>(std::forward<Args>(args)...);
                states.push_back(instance);
            }

            void addTransition(std::shared_ptr<Transition> transition); // Adds a transition to the manager
            void addTransition(std::string transition_name,std::string trigger_event,std::string source_state,std::string target_state); // Adds a transition to the manager

            void executeState(std::string state_name); // Executes the state with the given name
            void executeTransition(std::string transition_name); // Executes the transition with the given name

            bool checkStateExists(std::string state_name); // Checks if a state exists
            bool checkTransitionExists(std::string transition_name); // Checks if a transition exists

            virtual void executionFeedback(std::string executed_object_name)
            {
            }

            virtual void executionLoopBegin()
            {
            }

            virtual void executionLoopTerminated()
            {
            }

            virtual void logFeedback(std::string state_name,std::string log_type,std::string data)
            {
            }
            

            std::shared_ptr<State> getState(std::string state_name); // Returns a shared pointer to the state with the given name
            std::shared_ptr<Transition> getTransition(std::string transition_name); // Returns a shared pointer to the transition with the given name

            template <typename T> 
            void addParam(const std::string& name, T object); // Adds an object to the container with the given name
            
            template <typename T> 
            std::shared_ptr<T> getParam(const std::string& name); // Retrieves an object from the container with the given name

            void removeParam(const std::string& name); // Removes an object from the container with the given name

            template <typename T, typename... Args>
            static std::shared_ptr<StateManager> create(Args&&... args)
            {
                if (!state_manager)
                {
                    state_manager = std::make_shared<T>(std::forward<Args>(args)...);
                }

                return state_manager;
            }

            static void saveTree(const std::string& file_path, const std::string& file_name); // Saves the state tree to a file
            
            static std::shared_ptr<StateManager> state_manager; // Static shared pointer to the StateManager instance, can be used for global access

            
        private:
            std::vector<std::shared_ptr<State>> states; // List of all states
            std::vector<std::shared_ptr<Transition>> transitions; // List of all transitions
            std::vector<std::pair<std::string, std::shared_ptr<void>>> container; // Container for additional data, can be used for various purposes
    };

    class State
    {
        public:
            State(std::string name);
            ~State();
            std::shared_ptr<Transition> getTransitionFromEvent(std::string event); // Returns a transition based on the event
            void execute(std::shared_ptr<Transition> transition=nullptr); // Executes the state logic, calls onExecute

            virtual std::string onExecute(std::shared_ptr<Transition> transition) = 0;   // Pure virtual function to be implemented by derived classes

            std::string getName() const; // Returns the name of the state
            
            // Add this method to allow adding transitions
            void addTransition(std::shared_ptr<Transition> transition);

            void log(std::string log);
            void log_err(std::string log);
            void log_warn(std::string log);


        private: 
            std::string name;   // State name
            std::vector<std::shared_ptr<Transition>> sub_states; // Transitions to sub-states
            
    };

    class Transition : public std::enable_shared_from_this<Transition>
    {
        public:
            Transition(std::string name, std::string trigger, std::shared_ptr<State> source_state, std::shared_ptr<State> target_state);
            ~Transition();

            void execute(); // Executes the transition logic

            std::string getName() const; // Returns the name of the transition
            std::string getTrigger() const; // Returns the trigger condition for the transition
            std::shared_ptr<State> getSourceState() const; // Returns the source state
            std::shared_ptr<State> getTargetState() const; // Returns the target state

            void initialize(); // Method to set up connections after construction

        
        private:

            std::string name;   // Transition name
            std::string trigger;   // The word/condition that triggers the transition
            
            std::shared_ptr<State> source_state; // Pointer to the source state
            std::shared_ptr<State> target_state; // Pointer to the target state
    };

    template <typename T> 
    void StateManager::addParam(const std::string& name, T object) 
    {
        for (const auto& pair : container) {
            if (pair.first == name) {
                return;
            }
        }

        std::shared_ptr<T> ptr = std::make_shared<T>(std::move(object));
        container.emplace_back(name, ptr);
    }

    template <typename T> 
    std::shared_ptr<T> StateManager::getParam(const std::string& name) 
    {
        for (const auto& pair : container) {
            if (pair.first == name) {
                return std::static_pointer_cast<T>(pair.second);
            }
        }
        return nullptr;
    }

}