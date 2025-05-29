# Copyright 2025 Flytronic S.A.
from abc import ABC, abstractmethod
from enum import Enum, auto

class StateAction(Enum):
    CONTINUE = (auto(),)
    FINISHED = (auto(),)
    ABORT = (auto(),)


class State(ABC):
    def __init__(self, name: str) -> None:
        self._name = name

    @property
    def name(self) -> str:
        return self._name

    def __repr__(self) -> str:
        return f"State(name={self.name})"
    
    def __hash__(self) -> int:
        return hash(self.name)

    def __eq__(self, other: object) -> bool:
        if isinstance(other, State):
            return self.name == other.name
        raise NotImplementedError

    @abstractmethod
    def handle(self) -> Enum:
        raise NotImplementedError


Transitions = dict[State, dict[Enum, State]]


class EventNotExistForStateError(Exception):
    def __init__(self, state: State, event: Enum) -> None:
        message = f"{event!s} not exist for {state} in transition table"
        super().__init__(message)


class InvalidTransitionError(Exception):
    def __init__(self, state: State, event: Enum) -> None:
        super().__init__(f"Invalid transition for {state} with {event}")


class StateNotExistError(Exception):
    def __init__(self, state: State) -> None:
        message = f"{state!s} not exist in transition table"
        super().__init__(message)


class StateAttributeNotSetError(Exception):
    def __init__(self, state: State, attribute_name: str) -> None:
        super().__init__(f"Attribute {attribute_name} not set for {state}")


class StateMachine:
    def __init__(self, transitions: Transitions, state: State, event: Enum) -> None:
        self._transitions = transitions
        self._state = state
        self._event = event

    @property
    def state(self) -> State:
        return self._state

    @property
    def event(self) -> Enum:
        return self._event

    def set_event(self, event: Enum):
        if self.__is_event_exist_in_state(self._state, event):
            self._event = event
        else:
            raise EventNotExistForStateError(self._state, event)

    def handle(self) -> None:
        if not self.__is_state_exist(self._state):
            raise StateNotExistError(self._state)

        if self.__is_event_exist_in_state(self._state, self._event):
            self.__handle_transition()
        else:
            raise InvalidTransitionError(self._state, self._event)

    def __handle_transition(self):
        transition_state = self._transitions[self._state][self._event]
        self._event = transition_state.handle()
        self._state = transition_state

    def __is_state_exist(self, state: State) -> bool:
        return state in self._transitions

    def __is_event_exist_in_state(self, state: State, event: Enum) -> bool:
        return event in self._transitions[state]
