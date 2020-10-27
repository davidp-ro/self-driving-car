import cv2
import numpy as np
import sys
import warnings
from typing import Tuple

from utils import Utils

"""
Uses the Hough Lines Method to detect lanes.

 Atm tested with the video in 'test_data/test_video.mp4', and lane regions optimized
for a video that is 720x1280

Config options:
    previewHeight -> Height of the preview window, scale is maintained
    detectOuterLanes -> If false will skip detecting outer lanes
    ignoreRightOuterLine -> If true, ignore the right outer lane to avoid overlapping
    drawOuterLanes -> Show outer lanes (if detectOuterLanes is True)
    showMaskedImage -> Show masked images
    showGrayscaleImage -> Show the grayscale version of the original_image
    showConfigText -> Show the small config 'dump' on screen
    
Call:
    from detection import Detect
    detection = Detect(config)
    detection.detect(
        file_name -> Path of the video/image
        data_type -> "video" or "image"
    )
"""


class Detect:
    def __init__(self, cfg: dict):
        self.config: dict = cfg
        self.latest_distance_from_left: Tuple[int, int] = (0, 0)  # Lower, Upper
        self.latest_distance_from_right: Tuple[int, int] = (0, 0)  # Lower, Upper

    def detect(self, file_name: str, data_type: str) -> None:
        """
        Run the detection on a given file.

        :param file_name: Path of the video/image
        :param data_type: "video" or "image"
        """

        if data_type == "video":
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", category=RuntimeWarning)
                cap = cv2.VideoCapture(file_name)
                while cap.isOpened():
                    resp, frame = cap.read()
                    if not resp:  # EOF
                        print(f'Reached the end of the file.')
                        break
                    if cv2.waitKey(1) == ord('q'):  # If Q is pressed, quit the video
                        self._cleanup(cap)
                        break
                    self._analyzeFrame(frame, file_name)
        elif data_type == "image":
            image = np.copy(cv2.imread(file_name))
            self._analyzeFrame(image, file_name)
            if cv2.waitKey(0) == ord('q'):
                self._cleanup()
                sys.exit()
        sys.exit()

    def _analyzeFrame(self, current_frame: np.array, file_name: str = "Unknown filename") -> None:
        """
        Analyze a given frame

        :param current_frame: The current frame
        :param file_name: Optional, File name to be displayed on the title bar
        """
        canny = self._generateGradients(current_frame)

        # Inner lanes:
        cropped_image = self._regionOfInterest(canny)
        if self.config['showMaskedImage']:
            cv2.imshow("Masked Inner Lanes",
                       Utils.resizeWithAspectRatio(cropped_image, height=self.config['previewHeight']))
        found_lanes = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=40,
                                      maxLineGap=5)
        averaged_inner_lines = self._averageSlopeIntercept(current_frame, found_lanes)  # Smooth out lines

        # Outer lanes:
        if self.config['detectOuterLanes']:
            cropped_image = self._regionOfInterest(canny, whole_road=True)
            if self.config['showMaskedImage']:
                cv2.imshow("Masked Outer Lanes", Utils.resizeWithAspectRatio(
                    cropped_image,
                    height=self.config['previewHeight'])
                           )
            found_lanes = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=40,
                                          maxLineGap=5)
            averaged_outer_lines = self._averageSlopeIntercept(current_frame, found_lanes)  # Smooth out lines
        else:
            averaged_outer_lines = None

        if self.config['carPositionDetection']:
            self._checkCarPosition(current_frame, averaged_inner_lines, averaged_outer_lines)

        if not self.config['showPreviewWindow']:
            return  # Do not show the preview

        lanes_image = self._displayLanes(
            current_frame,
            inner_lanes=averaged_inner_lines,
            outer_lanes=averaged_outer_lines if self.config['drawOuterLanes'] else None
        )

        final_image = cv2.addWeighted(current_frame, 0.8, lanes_image, 1, 1)  # Blend lines into image

        cv2.line(final_image, (int(final_image.shape[1] / 2), final_image.shape[0]),
                 (int(final_image.shape[1] / 2), final_image.shape[0] - 10), (167, 88, 228), 25)

        final_image = Utils.resizeWithAspectRatio(final_image, height=self.config['previewHeight'])

        if self.config['showConfigText']:
            self._displayConfig(final_image)

        cv2.imshow(f'Lane Detection - {file_name} | Press Q to Quit', final_image)

    def _checkCarPosition(self, current_frame: np.array, inner_lane: np.array, outer_lanes: np.array) -> int:
        """
        Check the car position to keep the car in the center

            Keep the distance between the left line and the right line (from the center of the image, therefore car)
        equal. If no left lane is available, then keep a constant distance from the right line (keep the latest distance
        that was calculated when both lines were found. Same goes if a right line cannot be found, the left lane will
        be tracked. In case no lanes are found, returns 9 -> Stop the car

        TODO: If the car goes over the outer_lanes, stop.

        :param current_frame: The frame that is used when calculating
        :param inner_lane: Found inner lane (current lane)
        :param outer_lanes: Found outer lanes/lines (maximum "border" allowed)
        :returns:
            -1 -> The car is to the left of the center
            0 -> The car is in the center (within the margin of error)
            1 -> The car is to the right of the center
            9 -> The car went over the outer lines or something failed, stop the car
        """
        mid: int = int(current_frame.shape[0] / 2)
        left_lower, _, left_upper, _ = inner_lane[0]
        right_lower, _, right_upper, _ = inner_lane[1]
        if left_lower == 0 and right_lower == 0:
            # No lanes found, stop
            return 9
        elif left_lower == 0:
            # Left lane couldn't be found:
            if self.latest_distance_from_right == (0, 0):
                print('No data available, stop / unavailable')
                return 9
            else:
                print(
                    f'Left lane NOT found! Falling back to distance from right line: {self.latest_distance_from_right}')
                # TODO
        elif right_lower == 0:
            # Right lane couldn't be found:
            if self.latest_distance_from_left == (0, 0):
                print('No data available, stop / unavailable')
                return 9
            else:
                print(
                    f'Right lane NOT found! Falling back to distance from left line: {self.latest_distance_from_left}')
                # TODO
        else:
            # Both lanes found, update distances
            self.latest_distance_from_left = (mid - left_lower, mid - left_upper)
            self.latest_distance_from_right = (right_lower - mid, right_upper - mid)
            print(f'Both found, ldLeft={self.latest_distance_from_left} - ldRight={self.latest_distance_from_right}')

        return 0

    def _displayLanes(self, original_image: np.array, inner_lanes: np.array, outer_lanes: np.array) -> np.array:
        """
        Display the found lanes

        :param original_image: The original(normal) image
        :param inner_lanes: Found inner lanes, the lanes the car is travelling in
        :param outer_lanes: Found outer lanes, the margin of the road basically
        :return: The new image, black background with high-contrast lanes
        """
        lanes_image = np.zeros_like(original_image)

        if inner_lanes is not None:
            for lane in inner_lanes:
                x1, y1, x2, y2 = lane
                try:
                    cv2.line(lanes_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
                except OverflowError:
                    print(f'Overflow when drawing INNER lane!, coords: x1={x1}, y1={y1}, x2={x2}, y2={y2}')

        if outer_lanes is not None:
            for lane in outer_lanes:
                x1, y1, x2, y2 = lane
                if self.config['ignoreRightOuterLine'] and x1 > int(original_image.shape[1] / 2):
                    continue
                try:
                    cv2.line(lanes_image, (x1, y1), (x2, y2), (0, 0, 255), 5)
                except OverflowError:
                    print(f'Overflow when drawing OUTER lane!, coords: x1={x1}, y1={y1}, x2={x2}, y2={y2}')

        return lanes_image

    def _generateGradients(self, original_image: np.array) -> np.array:
        """
        Apply the Canny algorithm to the given image

        :param original_image: Original image
        :return: The gradients image
        """
        grayscale_image = cv2.cvtColor(original_image, cv2.COLOR_RGB2GRAY)  # Convert to grayscale
        grayscale_image = cv2.addWeighted(grayscale_image, 3, grayscale_image, 0, -75)  # Increase the contrast
        # Increasing the contrast to give me better precision
        # https://stackoverflow.com/questions/39308030/how-do-i-increase-the-contrast-of-an-image-in-python-opencv
        blurred_image = cv2.GaussianBlur(grayscale_image, (5, 5), 0)  # Apply Gaussian Blur to image to smooth out edges
        if self.config['showGrayscaleImage']:
            cv2.imshow("Greyscale", Utils.resizeWithAspectRatio(blurred_image, height=self.config['previewHeight']))
        return cv2.Canny(blurred_image, 50, 150)

    def _regionOfInterest(self, original_image: np.array, whole_road: bool = False) -> np.array:
        """
        Apply a mask to the image, and return the image after it was masked.

        :param original_image: Original image
        :param whole_road: If true returns the regions for the outer lines, if false, returns the inner ones
        :return: The image with the mask applied to it.
        """
        if not whole_road:
            return self._regionOfInterestForLane(original_image)
        else:
            return self._regionsOfInterestForRoad(original_image)

    @staticmethod
    def _regionOfInterestForLane(original_image: np.array) -> np.array:
        """
        Generates a mask that covers the current lane

        :param original_image: Original image
        :return: The image with the mask applied to it.
        """
        image_height = original_image.shape[0]  # 0 => Height | 1 => Width
        # Each polygon here is like this:
        #   [(lower_left, lower_right, upper_right, upper_left)]
        polygons = np.array([
            [(200, image_height), (1100, image_height), (800, image_height - 300), (500, int(image_height / 2))]
        ])
        mask = np.zeros_like(original_image)
        cv2.fillPoly(mask, polygons, 255)
        # cv2.imshow("MASK FOR LANE", mask)
        masked_image = cv2.bitwise_and(original_image, mask)
        return masked_image

    @staticmethod
    def _regionsOfInterestForRoad(original_image: np.array) -> np.array:
        """
        Generates a mask that covers the road margins

        :param original_image: Original image
        :return: The image with the mask applied to it.
        """
        image_height = original_image.shape[0]  # 0 => Height | 1 => Width
        midway = int(image_height / 2)
        # Each polygon here is like this:
        #   [(lower_left, lower_right, upper_right, upper_left)]
        polygons = np.array([
            [(0, image_height - 100), (300, image_height - 200), (250, midway), (50, midway + 150)],  # LEFT
            [(800, image_height), (1200, image_height), (1000, midway), (700, midway)]  # RIGHT
        ])
        mask = np.zeros_like(original_image)
        cv2.fillPoly(mask, polygons, 255)
        # cv2.imshow("MASK FOR ROAD", mask)
        masked_image = cv2.bitwise_and(original_image, mask)
        return masked_image

    @staticmethod
    def _getCoordinates(original_image: np.array, lane_average: np.array) -> np.array:
        """
        Get cartesian coordinates for the averaged lanes

        :param original_image: Original image
        :param lane_average: Averaged lane
        :return: [x1, y1, x2, y2] or [0, 0, 0, 0] if unpacking failed
        """
        try:
            slope, intercept = lane_average
            y1 = original_image.shape[0]  # Start from bottom
            y2 = int(y1 * (3 / 5))  # Go up to 3/5 of the image
            x1 = int((y1 - intercept) / slope)  # => x = (y-b)/m
            x2 = int((y2 - intercept) / slope)
            return np.array([x1, y1, x2, y2])
        except TypeError:
            # This happens when lane_average can't be unpacked,
            # for example when dealing with really short lines (ie. dashed)
            return np.array([0, 0, 0, 0])

    def _averageSlopeIntercept(self, original_image: np.array, lanes: np.array) -> np.array:
        """
        Average the lanes to get one smooth line

        :param original_image: Original image
        :param lanes: Lanes calculated with cv2.HoughLinesP
        :return: [left_lane, right_lane], where each lane is [x1, y1, x2, y2]
        """
        left_fit = []
        right_fit = []
        for lane in lanes:
            x1, y1, x2, y2 = lane.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)

        left_lane = self._getCoordinates(original_image, left_fit_average)
        right_lane = self._getCoordinates(original_image, right_fit_average)

        return np.array([left_lane, right_lane])

    def _displayConfig(self, original_image: np.array) -> None:
        """
        Display the config 'dump' on screen

        :param original_image: The original(normal) image
        """
        message = f"Config: detectOuter={self.config['detectOuterLanes']}&draw={self.config['drawOuterLanes']} ignoreRightOuter={self.config['ignoreRightOuterLine']}"
        cv2.putText(original_image, message,
                    (5, 20),
                    cv2.FONT_HERSHEY_DUPLEX, 0.7,
                    (255, 255, 255), 1)

    @staticmethod
    def _cleanup(cap: cv2.VideoCapture = None) -> None:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
