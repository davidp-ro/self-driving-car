"""
 brief: Part of the self-driving-car project, does all the detection and coordination.
 author: David Pescariu | https://github.com/davidp-ro

 copyright: GNU GPL v3 License
"""

__version__ = '1.3'

import numpy as np
import cv2
import warnings
from typing import Dict, Union
from controller import Controller
from utils.exceptions import ReachedEOF
from utils.utils import Utils


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
        # noinspection PyUnresolvedReferences
        print(f"[INIT] Numpy version: {np.__version__} | OpenCV version: {cv2.__version__}")

        self.controller = car_controller
        self.config = config
        self.thresholds = {
            'straight': 100,
            'light_turn': 150,
        }
        self.distance_to_one_lane = 230

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
                    frame: np.ndarray = cv2.flip(frame, 0)
                    frame = cv2.flip(frame, 1)
                    self._follow_lanes(frame)

    def _follow_lanes(self, frame: np.array) -> None:
        """
        Find the lanes, calculate the angle necessary to stay in the center,
        then send the commands to the controller.

        :param frame: Current frame
        """
        try:
            lanes = self._get_lanes(frame)
            angle = self._calculate_steering_angle(frame, lanes)
            if angle is None:
                self.controller.stop()
            elif angle == 0:
                self.controller.goForward()
            else:
                self.controller.turn(angle)
        except Exception as e:
            # ignore: too broad exception
            self.controller.stop()
            print(f'[FAIL::follow_lanes] {e}')

    def _get_lanes(self, frame: np.ndarray) -> np.ndarray:
        """
        Get the lanes (cartesian coords. relative to the frame)

        :param frame: Current frame to analyse
        :return: [left_lane, right_lane], where each lane is [x1, y1, x2, y2]
        """
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

    def _calculate_steering_angle(self, frame: np.ndarray, lanes: np.ndarray) -> Union[int, None]:
        """
        Calculate the steering angle necessary to keep the car in the center (equal distance between the lanes)

        :param frame: Current frame
        :param lanes: Found lanes
        :returns: 0 -> Go forward || -180 : 180 -> Turn || None -> Stop
        """
        try:
            # lanes => [x1, y1, x2, y2] aka [x_lwr, y_lwr, x_upr, y_upr]
            left_lane: np.ndarray = lanes[0]
            right_lane: np.ndarray = lanes[1]

            if len(lanes[0]) != 4 or len(lanes[1]) != 4:
                print('[WARN::calculate_steering_angle] No lanes found!')
                return None

            center = int(frame.shape[1] / 2)

            lwr_dst_to_the_left: int = abs(left_lane[0] - center)
            lwr_dst_to_the_right: int = abs(right_lane[0] - center)
            delta = lwr_dst_to_the_left - lwr_dst_to_the_right
            absolute_delta = abs(delta)
            
            print(f'[DISTANCES::calculate_steering_angle] l: {lwr_dst_to_the_left} r: {lwr_dst_to_the_right} d: {delta} ad: {absolute_delta}')

            if absolute_delta > 200:
                print(f'[INFO::calculate_steering_angle] Only one lane! Maintain d={self.distance_to_one_lane}')
                if lwr_dst_to_the_left > 1000:
                    # Maintain distance to the right lane
                    alpha = lwr_dst_to_the_right - self.distance_to_one_lane
                    print(f'[DEBUG::calculate_steering_angle] MAINTAIN RIGHT -> ALPHA {alpha}')
                    if abs(alpha) <= self.thresholds['straight']:
                        return 0
                    else:
                        return alpha
                else:
                    # Maintain distance to the left lane
                    alpha = lwr_dst_to_the_left - self.distance_to_one_lane
                    print(f'[DEBUG::calculate_steering_angle] MAINTAIN LEFT -> ALPHA {alpha}')
                    if abs(alpha) <= self.thresholds['straight']:
                        return 0
                    else:
                        return alpha

            if absolute_delta <= self.thresholds['straight']:
                return 0
            elif absolute_delta <= self.thresholds['light_turn']:
                return -1 * delta  # FIXME: Not correct, just for testing
            else:
                return -145 if delta < 0 else 145  # FIXME: Not 100% correct, just for testing
        except Exception as e:
            # FIXME: Remove broad exception
            print(f'[FAIL::calculate_steering_angle] {e}!')
            return None

    def _apply_canny(self, img: np.ndarray) -> np.ndarray:
        """
        Apply the Canny algorithm to the given image

        :param img: Original image
        :return: The gradients image
        """
        _grayscale_image = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)  # Convert to grayscale

        if self.config['greyscaleModifier'] != 0:
            # Increasing the contrast to give me better precision.
            # Note:
            #   It can actually make the image worse sometimes, so check
            #
            # TODO:
            #   Maybe add a light sensor and dynamically apply contrast?
            #
            # Src: https://stackoverflow.com/questions/39308030/how-do-i-increase-the-contrast-of-an-image-in-python-opencv
            _grayscale_image = cv2.addWeighted(_grayscale_image, 3, _grayscale_image, 0,
                                               self.config['greyscaleModifier'])

        _blurred_image = cv2.GaussianBlur(_grayscale_image, (5, 5), 0)  # Apply Gaussian Blur to image to smooth edges

        if self.config['showGrayscaleImage']:
            cv2.imshow("Greyscale", Utils.resizeWithAspectRatio(_blurred_image, height=self.config['previewHeight']))
        return cv2.Canny(_blurred_image, 50, 150)

    @staticmethod
    def _apply_mask_to_frame(img: np.ndarray) -> np.ndarray:
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
        _old_mask = np.array([
            # Left lane:
            [(0, _img_height), (175, _img_height), (250, _img_mid + 50), (175, _img_mid + 50)],

            # Right lane:
            [(_img_width - 175, _img_height), (_img_width, _img_height), (_img_width - 175, _img_mid + 50),
             (_img_width - 250, _img_mid + 50)]
        ])
        
        _mask = np.array([
            # left:
            [(0, _img_height), (175, _img_height), (250, _img_mid + 50), (0, _img_mid + 50)],
            
            # right:
            [(_img_width - 175, _img_height), (_img_width, _img_height), (_img_width, _img_mid + 50),
             (_img_width - 250, _img_mid + 50)]
        ])
        cv2.fillPoly(_masked, _mask, 255)
        # Show mask (for debug):
        # cv2.imshow("Lane mask", _masked)
        return cv2.bitwise_and(img, _masked)

    def _average_slope_intercept(self, img: np.ndarray, lines: np.ndarray) -> np.ndarray:
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
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
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
    def _get_coordinates(img: np.ndarray, lane_fit: np.ndarray) -> np.ndarray:
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
    def _display_lanes(img: np.ndarray, lanes: np.ndarray) -> np.ndarray:
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
                return _lanes_image
            try:
                cv2.line(_lanes_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
            except OverflowError:
                # print(f'Overflow when drawing INNER lane!, coords: x1={x1}, y1={y1}, x2={x2}, y2={y2}')
                print('[WARN::display_lanes] Overflow when drawing one of the lanes')

        return _lanes_image

    @staticmethod
    def _cleanup(cap: cv2.VideoCapture) -> None:
        cap.release()
        cv2.destroyAllWindows()
