"""
 brief: Part of the self-driving-car project, contains custom exceptions.
 author: David Pescariu | https://github.com/davidp-ro

 copyright: GNU GPL v3 License
"""

__version__ = '1.0'


class InvalidTurnArgument(Exception):
    def __init__(self, turn_angle: int):
        """
        If raised than the 'angle' argument is invalid.

        :param turn_angle: Given angle
        """
        self.msg = f"Angle {turn_angle} not in -180 : 180 valid range!"
        super().__init__(self.msg)


class ReachedEOF(Exception):
    def __init__(self):
        """
        Raised when cap.read() returns False as a response
        """
        super().__init__("Video feed down or reached EOF")
