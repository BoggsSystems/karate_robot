"""
Strategist state machine for the Karate Robot.
Bushido note: respect (Rei) and safety (Gi) precede action.
"""

from enum import Enum


class State(str, Enum):
    MOKUSO = "MOKUSO"       # Init / meditation
    REI = "REI"             # Auth / bow
    KAMAE = "KAMAE"         # Ready stance
    MIMIC = "MIMIC"         # Active imitation
    SAFETY_HALT = "SAFETY_HALT"


class IdentityGate:
    """
    Identity gatekeeper.
    Bushido note: the robot mirrors only the rightful Sensei.
    """

    def __call__(self) -> bool:
        return False


class StateMachine:
    def __init__(self, identity_gate=None):
        self.state = State.MOKUSO
        self.identity_gate = identity_gate or IdentityGate()

    def can_enter_mimic(self) -> bool:
        return bool(self.identity_gate())

    def transition(self, next_state: State) -> State:
        if self.state == State.SAFETY_HALT:
            # Bushido: in danger, stillness is the highest virtue.
            return self.state

        if next_state == State.MIMIC and not self.can_enter_mimic():
            # Respect before mimicry: refuse if the Sensei is not verified.
            return self.state

        self.state = next_state
        return self.state

    def safety_halt(self) -> State:
        self.state = State.SAFETY_HALT
        return self.state
