from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GripperControlConfig:
    close_torque: float
    hold_torque: float
    torque_limit: float
    move_kp: float
    move_kd: float
    close_kd: float
    stall_velocity: float
    stall_duration: float
    startup_distance: float

    def __post_init__(self) -> None:
        values = {
            "close_torque": self.close_torque,
            "hold_torque": self.hold_torque,
            "torque_limit": self.torque_limit,
            "move_kp": self.move_kp,
            "move_kd": self.move_kd,
            "close_kd": self.close_kd,
            "stall_velocity": self.stall_velocity,
            "stall_duration": self.stall_duration,
            "startup_distance": self.startup_distance,
        }
        if any(value < 0.0 for value in values.values()):
            raise ValueError("gripper control parameters must be non-negative")
        if self.torque_limit == 0.0:
            raise ValueError("gripper torque_limit must be positive")


@dataclass(frozen=True)
class GripperMitCommand:
    position: float
    velocity: float
    kp: float
    kd: float
    torque: float


class GripperControl:
    POSITION = "position"
    CLOSING = "closing"
    HOLDING = "holding"

    CONTACT = "contact"
    REACHED_TARGET = "reached_target"
    TIMEOUT = "timeout"
    CANCELED = "canceled"

    def __init__(
        self,
        open_position: float,
        close_position: float,
        goal_tolerance: float,
        config: GripperControlConfig,
    ) -> None:
        if open_position == close_position:
            raise ValueError("gripper open and close positions must differ")
        self.open_position = float(open_position)
        self.close_position = float(close_position)
        self.goal_tolerance = float(goal_tolerance)
        self.config = config
        self.close_sign = 1.0 if close_position > open_position else -1.0

        self.state = self.POSITION
        self.target_position = self.close_position
        self.start_position = self.close_position
        self.contact_position = self.close_position
        self.result: str | None = None
        self._active_close_torque = config.close_torque
        self._active_hold_torque = config.hold_torque
        self._stall_since: float | None = None

    def clamp_position(self, position: float) -> float:
        low = min(self.open_position, self.close_position)
        high = max(self.open_position, self.close_position)
        return max(low, min(high, float(position)))

    def is_closing_target(self, current: float, target: float) -> bool:
        target = self.clamp_position(target)
        return self.close_sign * (target - float(current)) > self.goal_tolerance

    def set_position(self, target: float, *, result: str | None = None) -> float:
        self.target_position = self.clamp_position(target)
        self.state = self.POSITION
        self.result = result
        self._stall_since = None
        return self.target_position

    def start_grasp(
        self,
        start_position: float,
        target_position: float,
        max_effort: float | None = None,
    ) -> None:
        self.start_position = float(start_position)
        self.contact_position = float(start_position)
        self.target_position = self.clamp_position(target_position)
        requested = (
            self.config.close_torque
            if max_effort is None or max_effort <= 0.0
            else abs(float(max_effort))
        )
        self._active_close_torque = min(requested, self.config.torque_limit)
        self._active_hold_torque = min(
            self.config.hold_torque,
            self._active_close_torque,
        )
        self.state = self.CLOSING
        self.result = None
        self._stall_since = None

        if self._target_reached(self.start_position):
            self.set_position(self.target_position, result=self.REACHED_TARGET)

    def cancel(self, current_position: float, result: str = CANCELED) -> None:
        self.set_position(current_position, result=result)

    def tick(
        self,
        position: float,
        velocity: float,
        now: float,
    ) -> GripperMitCommand:
        position = float(position)
        velocity = float(velocity)

        if self.state == self.CLOSING:
            self.contact_position = position
            moved = abs(position - self.start_position) >= self.config.startup_distance

            if self._target_reached(position):
                self.set_position(self.target_position, result=self.REACHED_TARGET)
                return self._position_command()

            if moved and abs(velocity) <= self.config.stall_velocity:
                if self._stall_since is None:
                    self._stall_since = float(now)
                elif now - self._stall_since >= self.config.stall_duration:
                    self.target_position = self.clamp_position(position)
                    self.contact_position = position
                    self.state = self.HOLDING
                    self.result = self.CONTACT
                    self._stall_since = None
                    return self._holding_command()
            else:
                self._stall_since = None

            return GripperMitCommand(
                position=self.target_position,
                velocity=0.0,
                kp=0.0,
                kd=self.config.close_kd,
                torque=self.close_sign * self._active_close_torque,
            )

        if self.state == self.HOLDING:
            return self._holding_command()

        return self._position_command()

    def _target_reached(self, position: float) -> bool:
        remaining = self.close_sign * (self.target_position - float(position))
        return remaining <= self.goal_tolerance

    def _position_command(self) -> GripperMitCommand:
        return GripperMitCommand(
            position=self.target_position,
            velocity=0.0,
            kp=self.config.move_kp,
            kd=self.config.move_kd,
            torque=0.0,
        )

    def _holding_command(self) -> GripperMitCommand:
        return GripperMitCommand(
            position=self.target_position,
            velocity=0.0,
            kp=self.config.move_kp,
            kd=self.config.move_kd,
            torque=self.close_sign * self._active_hold_torque,
        )
