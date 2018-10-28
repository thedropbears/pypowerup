class IMU:
    """Interface for our (saner) IMU classes."""

    def __init__(self) -> None:
        raise NotImplementedError

    def getAngle(self) -> float:
        """Get the current heading/yaw in radians.

        Angles are in the interval [-pi, pi], anticlockwise positive.
        """
        raise NotImplementedError

    def getHeadingRate(self) -> float:
        """Get the rate of change in yaw in radians per second."""
        raise NotImplementedError

    def resetHeading(self) -> None:
        """Zero the yaw."""
        raise NotImplementedError
