import math


def rescale_js(value, deadzone: float = 0.0, exponential: float = 0.0,
               rate: float = 1.0):
    """Rescale a joystick input, applying a deadzone, exponential, and max rate.

    Args:
        value: the joystick value, in the interval [-1, 1].
        deadzone: the deadzone to apply.
        exponential: the strength of the exponential to apply
                     (i.e. how non linear should the response be)
        rate: the max rate to return (ie the value to be returned when 1.0 is given)
    """
    sign = 1
    if value < 0:
        sign = -1
        value = -value
    # Apply deadzone
    if value < deadzone:
        return 0.0
    if not exponential:
        value = (value - deadzone) / (1 - deadzone)
    else:
        a = math.log(exponential + 1) / (1 - deadzone)
        value = (math.exp(a * (value - deadzone)) - 1) / exponential
    return rate * sign * value


def constrain_angle(angle):
    """Wrap an angle to the interval [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
