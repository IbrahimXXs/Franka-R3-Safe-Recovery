"""Pure withdrawal motion with fixed velocity and acceleration ramps.

For ordinary distances, all profiles have the same peak speed and speed-ramp
duration; only the constant-speed interval changes with withdrawal distance.
Short moves lower peak speed and shorten both ramps at the same v/r ratio.
"""
from dataclasses import dataclass
import math


def _number(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    if value < 0 or (positive and value == 0):
        raise ValueError(f'{name} must be {"positive" if positive else "nonnegative"}')
    return float(value)


@dataclass(frozen=True, init=False)
class WithdrawalProfile:
    distance_mm: float
    requested_speed_mm_s: float
    requested_ramp_s: float
    max_speed_mm_s: float
    ramp_s: float
    cruise_duration_s: float
    duration_s: float

    def __init__(self, distance_mm, speed_mm_s=5., ramp_s=.5):
        distance = _number(distance_mm, 'distance_mm')
        requested_speed = _number(speed_mm_s, 'speed_mm_s', positive=True)
        requested_ramp = _number(ramp_s, 'ramp_s', positive=True)
        if distance == 0:
            peak = ramp = cruise = duration = 0.
        elif distance < requested_speed * requested_ramp:
            # sqrt(D) * sqrt(r/v) avoids multiplying a tiny distance by r first.
            ramp = math.sqrt(distance) * math.sqrt(requested_ramp / requested_speed)
            peak = requested_speed * ramp / requested_ramp
            cruise, duration = 0., 2. * ramp
        else:
            peak, ramp = requested_speed, requested_ramp
            cruise = max(0., distance / peak - ramp)
            duration = distance / peak + ramp
        for name, value in dict(distance_mm=distance, requested_speed_mm_s=requested_speed,
                                requested_ramp_s=requested_ramp, max_speed_mm_s=peak,
                                ramp_s=ramp, cruise_duration_s=cruise, duration_s=duration).items():
            object.__setattr__(self, name, value)

    @property
    def effective_ramp_s(self):
        return self.ramp_s

    @staticmethod
    def _time(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('Profile time must be finite')
        return float(value)

    def _ramp_position(self, elapsed_s):
        fraction = elapsed_s / self.ramp_s
        return self.max_speed_mm_s * self.ramp_s * (fraction**3 - .5 * fraction**4)

    def position_mm(self, elapsed_s):
        time = self._time(elapsed_s)
        if time <= 0 or self.distance_mm == 0:
            return 0.
        if time >= self.duration_s:
            return self.distance_mm
        if time <= self.ramp_s:
            return self._ramp_position(time)
        if time < self.duration_s - self.ramp_s:
            return .5 * self.max_speed_mm_s * self.ramp_s + self.max_speed_mm_s * (time - self.ramp_s)
        return self.distance_mm - self._ramp_position(self.duration_s - time)

    def speed_mm_s(self, elapsed_s):
        time = self._time(elapsed_s)
        if time <= 0 or time >= self.duration_s or self.distance_mm == 0:
            return 0.
        if time < self.ramp_s:
            fraction = time / self.ramp_s
        elif time <= self.duration_s - self.ramp_s:
            return self.max_speed_mm_s
        else:
            fraction = (self.duration_s - time) / self.ramp_s
        return self.max_speed_mm_s * fraction * fraction * (3. - 2. * fraction)

    @property
    def metadata(self):
        return dict(profile='smoothstep_velocity_ramps_v1', distance_mm=self.distance_mm,
                    requested_speed_mm_s=self.requested_speed_mm_s, requested_ramp_s=self.requested_ramp_s,
                    max_speed_mm_s=self.max_speed_mm_s, effective_ramp_s=self.ramp_s,
                    cruise_duration_s=self.cruise_duration_s, duration_s=self.duration_s,
                    short_move=0 < self.distance_mm < self.requested_speed_mm_s * self.requested_ramp_s,
                    max_acceleration_mm_s2=1.5 * self.max_speed_mm_s / self.ramp_s if self.ramp_s else 0.)

    def as_dict(self):
        return self.metadata
