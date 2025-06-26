#include <easysm/core.hpp>
#include <boost/smart_ptr/enable_shared_from_this.hpp>
#include <fstream>

namespace easysm
{
    StateManager::StateManager() 
    {

    }

    StateManager::~StateManager() 
    {

    }

    std::shared_ptr<StateManager> StateManager::state_manager = nullptr;

    void StateManager::saveTree(const std::string& file_path, const std::string& file_name) 
    {
        if (!state_manager) {
            return; // No state manager instance available
        }

        // Create the complete file path with .easysm_tree extension
        std::string full_path = file_path;
        if (!full_path.empty() && full_path.back() != '/') {
            full_path += "/";
        }
        full_path += file_name + ".easysm_tree";
        
        // Open file for writing (will overwrite if exists, create if doesn't exist)
        std::ofstream file(full_path);
        
        if (!file.is_open()) {
            return; // Could not open file for writing
        }
        
        // Write all states
        for (const auto& state : state_manager->states) {
            file << "node//" << state->getName() << "//400//400\n";
        }
        
        // Write all transitions
        for (const auto& transition : state_manager->transitions) {
            auto source_state = transition->getSourceState();
            auto target_state = transition->getTargetState();
            
            if (source_state && target_state) {
                file << "transition//" << transition->getName() 
                     << "//" << source_state->getName() 
                     << "//" << target_state->getName() << "\n";
            }
        }
        
        file.close();
    }

    void StateManager::addState(std::shared_ptr<State> state) 
    {
        if(!checkStateExists(state->getName())) 
        {
            states.push_back(state);
            return;
        }

    }

    void StateManager::addTransition(std::shared_ptr<Transition> transition) 
    {
        if(!checkTransitionExists(transition->getName())) 
        {
            transitions.push_back(transition);
            transition->initialize(); 
            return;
        }

    }

    void StateManager::addTransition(std::string transition_name,std::string trigger_event,std::string source_state,std::string target_state)
    {
        if(!checkTransitionExists(transition_name)) 
        {
            auto source = getState(source_state);
            auto target = getState(target_state);
            if (source && target)
            {
                auto transition = std::make_shared<easysm::Transition>(transition_name, trigger_event, source, target);
                transitions.push_back(transition);
                transition->initialize(); 
                return;
            }
        }
    }


    void StateManager::executeState(std::string state_name) 
    {
        executionLoopBegin();
        for (const auto& state : states) {
            if (state->getName() == state_name) {
                state->execute(nullptr);
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

    void StateManager::removeParam(const std::string& name) 
    {
        for (auto it = container.begin(); it != container.end(); ++it) {
            if (it->first == name) {
                container.erase(it);
                return;
            }
        }
    }

    State::State(std::string name) : name(name)
    {

    }

    State::~State() 
    {
                 
    }

    void State::execute(std::shared_ptr<Transition> transition) 
    {
        StateManager::state_manager->executionFeedback(name); 
        std::string event = onExecute(transition); // Burda patlıyor
        std::shared_ptr<Transition> child_transition = getTransitionFromEvent(event);

        if (child_transition) 
        {
            child_transition->execute();
        }
        else
        {
            StateManager::state_manager->executionLoopTerminated();
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

    void State::log(std::string log) 
    {
        StateManager::state_manager->logFeedback(name,"log",log);
    }

    void State::log_err(std::string log) 
    {
        StateManager::state_manager->logFeedback(name,"log_error",log);
    }

    void State::log_warn(std::string log) 
    {
        StateManager::state_manager->logFeedback(name,"log_warn",log);
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
        StateManager::state_manager->executionFeedback(name);
        target_state->execute(this->shared_from_this());
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

    std::shared_ptr<State> Transition::getSourceState() const 
    {
        return source_state;
    }

    std::shared_ptr<State> Transition::getTargetState() const 
    {
        return target_state;
    }

}