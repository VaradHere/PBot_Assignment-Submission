#!/usr/bin/env python3
"""
Finite State Machine state definitions and transitions for P-Bot AGV
"""

from enum import Enum


class FSMState(Enum):
    """AGV operational states"""
    IDLE = "IDLE"
    AWAITING_ORDER = "AWAITING_ORDER"
    NAVIGATING_TO_PICKUP = "NAVIGATING_TO_PICKUP"
    PICKUP_ACTION = "PICKUP_ACTION"
    NAVIGATING_TO_DELIVERY = "NAVIGATING_TO_DELIVERY"
    DROPOFF_ACTION = "DROPOFF_ACTION"
    RETURNING_TO_DOCK = "RETURNING_TO_DOCK"
    ERROR_RECOVERY = "ERROR_RECOVERY"


class FSMTransitionError(Exception):
    """Raised when an invalid state transition is attempted"""
    pass


class FSMManager:
    """
    Manages FSM state transitions with validation
    Ensures deterministic and traceable state flow
    """
    
    # Valid state transitions (from_state -> [valid_to_states])
    VALID_TRANSITIONS = {
        FSMState.IDLE: [FSMState.AWAITING_ORDER],
        FSMState.AWAITING_ORDER: [FSMState.NAVIGATING_TO_PICKUP, FSMState.ERROR_RECOVERY, FSMState.IDLE],
        FSMState.NAVIGATING_TO_PICKUP: [FSMState.PICKUP_ACTION, FSMState.ERROR_RECOVERY],
        FSMState.PICKUP_ACTION: [FSMState.NAVIGATING_TO_DELIVERY, FSMState.ERROR_RECOVERY],
        FSMState.NAVIGATING_TO_DELIVERY: [FSMState.DROPOFF_ACTION, FSMState.ERROR_RECOVERY],
        FSMState.DROPOFF_ACTION: [FSMState.RETURNING_TO_DOCK, FSMState.ERROR_RECOVERY],
        FSMState.RETURNING_TO_DOCK: [FSMState.IDLE, FSMState.ERROR_RECOVERY],
        FSMState.ERROR_RECOVERY: [FSMState.RETURNING_TO_DOCK, FSMState.IDLE],
    }
    
    def __init__(self, logger, initial_state=FSMState.IDLE):
        self.current_state = initial_state
        self.previous_state = None
        self.state_history = [initial_state]
        self.logger = logger
        self.transition_count = 0
    
    def transition_to(self, new_state: FSMState, reason: str = ""):
        """
        Perform validated state transition
        
        Args:
            new_state: Target FSM state
            reason: Human-readable reason for transition
        
        Raises:
            FSMTransitionError: If transition is invalid
        """
        if new_state not in self.VALID_TRANSITIONS.get(self.current_state, []):
            error_msg = (
                f"Invalid transition: {self.current_state.value} -> {new_state.value}. "
                f"Valid transitions: {[s.value for s in self.VALID_TRANSITIONS.get(self.current_state, [])]}"
            )
            self.logger.error(error_msg)
            raise FSMTransitionError(error_msg)
        
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_history.append(new_state)
        self.transition_count += 1
        
        log_msg = f"[FSM] Transition #{self.transition_count}: {self.previous_state.value} → {new_state.value}"
        if reason:
            log_msg += f" | Reason: {reason}"
        
        self.logger.info(log_msg)
    
    def get_current_state(self) -> FSMState:
        """Return current FSM state"""
        return self.current_state
    
    def get_state_name(self) -> str:
        """Return current state as string"""
        return self.current_state.value
    
    def is_in_state(self, state: FSMState) -> bool:
        """Check if FSM is in specified state"""
        return self.current_state == state
    
    def can_accept_order(self) -> bool:
        """Check if robot can accept new orders"""
        return self.current_state == FSMState.IDLE
    
    def is_mission_active(self) -> bool:
        """Check if a mission is currently executing"""
        return self.current_state not in [FSMState.IDLE, FSMState.ERROR_RECOVERY]
