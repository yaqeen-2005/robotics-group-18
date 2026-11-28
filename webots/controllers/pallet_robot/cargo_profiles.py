"""Provisional cargo-dependent motion limits, in SI units."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CargoProfile:
    max_speed: float
    max_acceleration: float
    max_deceleration: float
    max_turn_speed: float
    max_angular_acceleration: float

    def __post_init__(self):
        limits = (self.max_speed, self.max_acceleration,
                  self.max_deceleration, self.max_turn_speed,
                  self.max_angular_acceleration)
        if not all(math.isfinite(value) and value > 0 for value in limits):
            raise ValueError('All motion limits must be finite and positive')


CARGO_PROFILES = {
    'empty': CargoProfile(0.60, 0.50, 0.50, 0.50, 1.00),
    'standard': CargoProfile(0.45, 0.30, 0.30, 0.40, 0.60),
    'wide': CargoProfile(0.40, 0.25, 0.25, 0.30, 0.45),
    'fragile': CargoProfile(0.25, 0.15, 0.15, 0.20, 0.30),
}
