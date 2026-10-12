#include "FSM/FSM.h"
#include <iostream>  


FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){
    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.loco = new State_Loco(_ctrlComp);
    _stateList.amp = new State_AMP(_ctrlComp);
    _stateList.mjamp = new State_MJAMP(_ctrlComp);
    _stateList.mjampRecovery = new State_MJAMP_RECOVERY(_ctrlComp);
    if (_ctrlComp->locoAutoRecovery)
        _stateList.locoRecovery = new State_LocoRecovery(
            _ctrlComp, *_stateList.loco, *_stateList.mjampRecovery);
    _stateList.wbc = new State_WBC(_ctrlComp);
    initialize(); 
}

FSM::~FSM(){  
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();  
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;  

    std::cout<<"[FSM] Initial state: PASSIVE (damping). Recovery does not require START."<<std::endl;
    std::cout<<"Press **R2+X** for recovery damping wait; release buttons, then **R2+Y** to start."<<std::endl;
    std::cout<<"**START** is optional and enters the fixed standing pose, not recovery."<<std::endl;
    if (_ctrlComp->locoAutoRecovery)
        std::cout << "[Task] RL Loco+MJAmpRecovery: START, wait for standing, then R2+B. "
                     "All LOCO entries use automatic recovery. Center sticks to enable walking." << std::endl;
}

void FSM::run(){
    try{
        _startTime = getSystemTime();  
        
        _ctrlComp->sendRecv(); 

        // In the new task, manual stop also cancels a pending outer transition.
        if (_ctrlComp->locoAutoRecovery &&
            (_ctrlComp->lowState->userCmd == UserCommand::L2_B ||
             _ctrlComp->lowState->userCmd == UserCommand::SELECT)) {
            _currentState->exit();
            _currentState = _stateList.passive;
            _currentState->enter();
            _nextState = _currentState;
            _mode = FSMMode::NORMAL;
            if (_ctrlComp->lowState->userCmd == UserCommand::SELECT)
                throw std::runtime_error("exit..");
        }

        if(_mode == FSMMode::NORMAL){  
            _currentState->run();  
            _nextStateName = _currentState->checkChange();    
            if (_ctrlComp->locoAutoRecovery && _nextStateName == FSMStateName::LOCO)
                _nextStateName = FSMStateName::LOCO_RECOVERY;
            if(_nextStateName != _currentState->_stateName){  
                _mode = FSMMode::CHANGE;  
                _nextState = getNextState(_nextStateName); 
                std::cout << "Switched from " << _currentState->_stateNameString
                << " to " << _nextState->_stateNameString << std::endl; 
            }
        }
        else if(_mode == FSMMode::CHANGE){  
            _currentState->exit();  
            _currentState = _nextState; 
            _currentState->enter();  
            _mode = FSMMode::NORMAL; 
            _currentState->run(); 
        }

        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));  
    }catch (const std::exception& e) {
        std::cerr << std::endl << "Caught exception: " << e.what() << std::endl;
        if (_currentState->_stateName == FSMStateName::MJAMP_RECOVERY ||
            _ctrlComp->locoAutoRecovery) {
            // SELECT/exception ends the main loop before its next sendRecv().
            // Publish damping now instead of leaving the last policy target active.
            _stateList.passive->enter();
            _ctrlComp->sendRecv();
        }
        _ctrlComp->exitFlag = true;
    }
}

FSMState* FSM::getNextState(FSMStateName stateName){  
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::LOCO: 
        return _stateList.loco;
    case FSMStateName::WBC:
        return _stateList.wbc;
    case FSMStateName::AMP:
        return _stateList.amp;
    case FSMStateName::MJAMP:
        return _stateList.mjamp;
    case FSMStateName::MJAMP_RECOVERY:
        return _stateList.mjampRecovery;
    case FSMStateName::LOCO_RECOVERY:
        if (!_stateList.locoRecovery) throw std::runtime_error("Automatic recovery is not enabled");
        return _stateList.locoRecovery;
    default:
        return _stateList.invalid;
        break;
    }
}
