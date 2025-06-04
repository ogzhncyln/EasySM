#include <core.hpp>
#include <boost/smart_ptr/enable_shared_from_this.hpp>

namespace easysm
{
    StateManager::StateManager() 
    {

    }

    StateManager::~StateManager() 
    {

    }

    void StateManager::addState(std::shared_ptr<State> state) 
    {
        if(!checkStateExists(state->getName())) 
        {
            states.push_back(state);
            state->state_manager = shared_from_this(); 
            return;
        }

    }

    void StateManager::addTransition(std::shared_ptr<Transition> transition) 
    {
        if(!checkTransitionExists(transition->getName())) 
        {
            transitions.push_back(transition);
            transition->state_manager = shared_from_this(); 
            transition->initialize(); 
            return;
        }

    }

    void StateManager::executeState(std::string state_name) 
    {
        for (const auto& state : states) {
            if (state->getName() == state_name) {
                state->execute();
                return;
            }
        }
    }

    void StateManager::executeTransition(std::string transition_name) 
    {
        for (const auto& transition : transitions) {
            if (transition->getName() == transition_name) {
                transition->execute();
                return;
            }
        }
    }

    bool StateManager::checkStateExists(std::string state_name) 
    {
        for (const auto& state : states) {
            if (state->getName() == state_name) {
                return true;
            }
        }
        return false;
    }

    bool StateManager::checkTransitionExists(std::string transition_name) 
    {
        for (const auto& transition : transitions) {
            if (transition->getName() == transition_name) {
                return true;
            }
        }
        return false;
    }

    std::shared_ptr<State> StateManager::getState(std::string state_name) 
    {
        for (const auto& state : states) {
            if (state->getName() == state_name) {
                return state;
            }
        }
        return nullptr;
    }

    std::shared_ptr<Transition> StateManager::getTransition(std::string transition_name) 
    {
        for (const auto& transition : transitions) {
            if (transition->getName() == transition_name) {
                return transition;
            }
        }
        return nullptr;
    }

    template void StateManager::addParam<std::vector<int>>(const std::string&, std::vector<int>);
    template std::shared_ptr<std::vector<int>> StateManager::getParam<std::vector<int>>(const std::string&);

    State::State(std::string name) : name(name)
    {

    }

    State::~State() 
    {

    }

    void State::execute() 
    {
        state_manager->executionFeedback(name); 
        std::string event = onExecute(); 
        std::shared_ptr<Transition> transition = getTransitionFromEvent(event);

        if (transition) 
        {
            transition->execute();
        }
        else
        {
            state_manager->executionLoopTerminated();
        }

    }

    std::shared_ptr<Transition> State::getTransitionFromEvent(std::string event) 
    {
        for (const auto& transition : sub_states) {
            if (transition->getTrigger() == event) {
                return transition;
            }
        }
        return nullptr;
    }

    void State::addTransition(std::shared_ptr<Transition> transition)
    {
        sub_states.push_back(transition);
    }

    std::string State::getName() const 
    {
        return name;
    }

    Transition::Transition(std::string name, std::string trigger, std::shared_ptr<State> source_state, std::shared_ptr<State> target_state) 
        : name(name), trigger(trigger), source_state(source_state), target_state(target_state) 
    {

    }

    Transition::~Transition() 
    {

    }

    void Transition::execute() 
    {
        state_manager->executionFeedback(name);
        target_state->execute(); 
    }

    void Transition::initialize()
    {
        source_state->addTransition(shared_from_this());
    }

    std::string Transition::getName() const 
    {
        return name;
    }

    std::string Transition::getTrigger() const 
    {
        return trigger;
    }
}