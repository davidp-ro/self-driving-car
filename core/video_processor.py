import numpy as np
import cv2
import warnings
from typing import Dict
from controller import Controller
from exceptions import ReachedEOF


class VideoProcessor:
    def __init__(self, config: Dict, car_controller: Controller):
        """
        Process frames from the pi camera.

        :param config: Config dictionary
        :param car_controller: Controller instance
        """
        cv2_version = cv2.version
        print(f"[INIT] Numpy version: {np.__version__} | OpenCV version: {cv2_version}")

        self.controller = car_controller
        self.config = config

    def run(self) -> None:
        """
        Start processing frames

        :raises: ReachedEOF if the feed stops
        """
        self.controller.stop()

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            capture = cv2.VideoCapture(0)

            while capture.isOpened():
                res, frame = capture.read()
                if not res:
                    raise ReachedEOF()
                if cv2.waitKey(1) == ord('q'):  # If Q is pressed, quit the video
                    # TODO: Cleanup when quiting
                    break

                # Tasks for each frame:
                self._follow_lanes(frame)

    def _follow_lanes(self, frame: np.array):
        pass
