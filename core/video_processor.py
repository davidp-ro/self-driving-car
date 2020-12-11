"""
 brief: Part of the self-driving-car project, does all the detection and coordination.
 author: David Pescariu | https://github.com/davidp-ro

 copyright: GNU GPL v3 License
"""

__version__ = '1.0'

import numpy as np
import cv2
import warnings
from typing import Dict
from core.controller import Controller
from core.utils.exceptions import ReachedEOF
from core.utils.utils import Utils


class VideoProcessor:
    """
    This does everything from finding the lanes, to giving directions to the controller,
    to showing the images.

    How it works?
        For each frame (in functions it's the `img` parameter) it will firstly find the lanes.
        Then it will calculate the steering angle necessary to keep the car in the center.
            -> None => Stop
            -> 0 => Go forward
            -> Other values => Turn
        And finally sent commands to the controller.
    """
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
                elif cv2.waitKey(1) == ord('q'):  # If Q is pressed, quit the video
                    self._cleanup(capture)
                    break
                else:
                    # Tasks for frame by frame
                    frame = cv2.flip(frame, 0)
                    frame = cv2.flip(frame, 1)
                    self._follow_lanes(frame)

    def _follow_lanes(self, frame: np.array):
        """
        TODO: Docs
        :param frame:
        :return:
        """
        lanes = self._get_lanes(frame)
        angle = self._calculate_steering_angle(frame, lanes)
        if angle is None:
            self.controller.stop()
        elif angle == 0:
            self.controller.goForward()
        else:
            self.controller.turn(angle)

    def _get_lanes(self, frame: np.array) -> np.array:
        # Do canny edge detection
        canny = self._apply_canny(frame)

        # Apply the mask over the image
        cropped_frame = self._apply_mask_to_frame(canny)

        # Find lines in the image using Probabilistic Hough Transform
        found_lines = cv2.HoughLinesP(cropped_frame, 2, np.pi / 180, 100, np.array([]),
                                      minLineLength=40,
                                      maxLineGap=5)

        # Finally average the lines to get the actual lanes
        lanes = self._average_slope_intercept(frame, found_lines)

        if not self.config['enableDebugImages']:
            return lanes
        else:
            # Show images for debug:
            if self.config['showMaskedImage']:
                cv2.imshow("Masked Lanes",
                           Utils.resizeWithAspectRatio(cropped_frame, height=self.config['previewHeight']))

            # Image with lanes on it:
            _lanes_image = self._display_lanes(frame, lanes)
            _final_image = cv2.addWeighted(frame, 0.8, _lanes_image, 1, 1)  # Blend lanes into image

            _final_image = Utils.resizeWithAspectRatio(_final_image, height=self.config['previewHeight'])
            cv2.imshow(f'VideoProcessor Preview - Version {__version__}', _final_image)
            return lanes

    def _calculate_steering_angle(self, frame: np.array, lanes: np.array) -> int or None:
        """
        For each frame, find the lanes, calculate the steering angle.

        TODO: Docs
        :param frame:
        :param lanes:
        :return:
        """
        return 0

    @staticmethod
    def _apply_canny(img: np.array) -> np.array:
        """
        Apply the Canny algorithm to the given image

        :param img: Original image
        :return: The gradients image
        """
        _grayscale_image = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)  # Convert to grayscale
        # Increase the contrast:
        _grayscale_image = cv2.addWeighted(_grayscale_image, 3, _grayscale_image, 0, self.config['greyscaleModifier'])
        # Increasing the contrast to give me better precision
        # https://stackoverflow.com/questions/39308030/how-do-i-increase-the-contrast-of-an-image-in-python-opencv
        _blurred_image = cv2.GaussianBlur(_grayscale_image, (5, 5), 0)  # Apply Gaussian Blur to image to smooth out edges
        if self.config['showGrayscaleImage']:
            cv2.imshow("Greyscale", Utils.resizeWithAspectRatio(_blurred_image, height=self.config['previewHeight']))
        return cv2.Canny(_blurred_image, 50, 150)

    @staticmethod
    def _apply_mask_to_frame(img: np.array) -> np.array:
        """
        Apply the mask to each frame

        :param img: Original image
        :return: Image with the mask applied to it
        """
        _masked = np.zeros_like(img)
        _img_height = img.shape[0]
        _img_width = img.shape[1]
        _img_mid = int(_img_height / 2)
        # Mask :: [(lower_left, lower_right, upper_right, upper_left)]
        _mask = np.array([
            [(100, _img_height), (_img_width - 100, _img_height), (400, _img_mid + 100), (200, _img_mid + 100)]
        ])
        cv2.fillPoly(_masked, _mask, 255)
        # Show mask (for debug):
        # cv2.imshow("Lane mask", _mask)
        return cv2.bitwise_and(img, _masked)

    def _average_slope_intercept(self, img: np.array, lines: np.array) -> np.array:
        """
        Average the lanes to get one smooth line

        :param img: Original image
        :param lines: Lines calculated with HoughLinesP
        :returns: The lanes -> [left_lane, right_lane], where each lane is [x1, y1, x2, y2]
        """
        _left_fit = []
        _right_fit = []
        if lines is None:
            return np.array([(0, 0), (0, 0)])
        for x1, y1, x2, y2 in lines:
            _parameters = np.polyfit((x1, x2), (y1, y2), 1)
            _slope = _parameters[0]
            _intercept = _parameters[1]
            if _slope < 0:
                _left_fit.append((_slope, _intercept))
            else:
                _right_fit.append((_slope, _intercept))

        _left_fit_average = np.average(_left_fit, axis=0)
        _right_fit_average = np.average(_right_fit, axis=0)

        left_lane = self._get_coordinates(img, _left_fit_average)
        right_lane = self._get_coordinates(img, _right_fit_average)

        return np.array([left_lane, right_lane])

    @staticmethod
    def _get_coordinates(img: np.array, lane_fit: np.array) -> np.array:
        """
        Get cartesian coordinates for the averaged lines

        :param img: Original image
        :param lane_fit: Averaged line
        :return: [x1, y1, x2, y2] or [0, 0, 0, 0] if unpacking failed
        """
        try:
            _slope, _intercept = lane_fit
            y1 = img.shape[0]  # Start from bottom
            y2 = int(y1 * 4 / 5)  # Go up to 3/5 of the image
            x1 = int((y1 - _intercept) / _slope)  # => x = (y-b)/m
            x2 = int((y2 - _intercept) / _slope)
            return np.array([x1, y1, x2, y2])
        except TypeError:
            # This happens when lane_average can't be unpacked,
            # for example when dealing with really short lines (ie. dashed)
            return np.array([0, 0, 0, 0])
        except OverflowError:
            return np.array([0, 0, 0, 0])

    @staticmethod
    def _display_lanes(img: np.array, lanes: np.array) -> np.array:
        """
        Generate an image with the lanes on it. Also draws the center line

        :param img: Original image
        :param lanes: Found lanes
        :return: Image with lanes on it
        """
        _lanes_image = np.zeros_like(img)

        cv2.line(_lanes_image, (int(_lanes_image.shape[1] / 2), _lanes_image.shape[0]),
                 (int(_lanes_image.shape[1] / 2), _lanes_image.shape[0] - 10), (167, 88, 228), 25)

        for lane in lanes:
            try:
                x1, y1, x2, y2 = lane
            except ValueError:
                print('[WARN] No lanes found!')
                return _lanes_image
            try:
                cv2.line(_lanes_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
            except OverflowError:
                print(f'Overflow when drawing INNER lane!, coords: x1={x1}, y1={y1}, x2={x2}, y2={y2}')

        return _lanes_image

    @staticmethod
    def _cleanup(cap: cv2.VideoCapture) -> None:
        cap.release()
        cv2.destroyAllWindows()
