"""
 brief: Part of the self-driving-car project, deals with communicating to the UNO Controller.
 author: David Pescariu | https://github.com/davidp-ro

 copyright: GNU GPL v3 License
"""

__version__ = '1.1'

import serial
from utils.exceptions import InvalidTurnArgument, InvalidSpeed


class Controller:
    """
    APIs for communicating with the UNO Controller.
    """
    def __init__(self,
                 port: str,
                 baudrate: int = 9600,
                 timeout: int = 1,
                 encoding: str = 'utf-8',
                 terminator: str = '\n'):
        """
        Initialize the communication between the pi and the controller

        :param port: /dev/tty*
        :param baudrate: Set baudrate, defaults to 9600
        :param timeout: Timeout in seconds, defaults to 1 second
        :param encoding: Encoding for the commands, defaults to utf 8
        :param terminator: Command terminator (for readStringUntil), defaults to \n
        """
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.encoding = encoding
        self.terminator = terminator

        self.ser.flush()

    def _write(self, command: str) -> None:
        """
        Send command via serial

        :param command: The command
        """
        _command = f'{command.replace(" ", "")}{self.terminator}'
        self.ser.write(_command.encode(self.encoding))

    def set_speed(self, speed: str) -> None:
        """
        Set the motor speed

        :param speed: slow / med / fast

        :raises: InvalidSpeed if the speed is invalid
        """
        if speed in ['slow', 'med', 'fast']:
            self._write(f'set_speed={speed}')
        else:
            raise InvalidSpeed(speed)

    def stop(self) -> None:
        """
        Stop the car
        """
        self._write('stop')

    def goForward(self) -> None:
        """
        Start going forward
        """
        self._write('fwd')

    def reverse(self) -> None:
        """
        Reverse car
        """
        self._write('rev')

    def turn(self, angle: int) -> None:
        """
        Execute a turn

        :param angle: If less than 0 turn left else turn right

        :raises: InvalidTurnArgument if angle is < -180 or > 180
        """
        if not (-180 < angle < 180):
            raise InvalidTurnArgument(angle)
        if angle > 0:
            # Right turn
            if angle < 75:
                self._write('turn_sr')
            else:
                self._write('turn_r')
        else:
            # Left turn
            if angle > -75:
                self._write('turn_sl')
            else:
                self._write('turn_l')
