import cv2
import numpy as np
import sys
import warnings

config = {
    'previewHeight': 480,
}


def detect(file_name: str, data_type: str = "video"):
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
                    _cleanup(cap)
                    break
                analyzeFrame(frame, file_name)
    elif data_type == "image":
        image = np.copy(cv2.imread(file_name))
        analyzeFrame(image, file_name)
        if cv2.waitKey(0) == ord('q'):
            _cleanup()
            sys.exit()
    sys.exit()


def analyzeFrame(current_frame, file_name: str = "Unknown filename"):
    canny = _generateGradients(current_frame)
    cropped_image = _regionOfInterest(canny)

    found_lanes = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=5)
    averaged_lines = _averageSlopeIntercept(current_frame, found_lanes)

    lanes_image = displayLanes(current_frame, averaged_lines)
    final_image = cv2.addWeighted(current_frame, 0.8, lanes_image, 1, 1)

    final_image = _resizeWithAspectRatio(final_image, height=config['previewHeight'])

    cv2.imshow(f'Lane Detection - {file_name} | Press Q to Quit', final_image)


def displayText(original_image: np.array, message: str):
    cv2.putText(original_image, message, (5, original_image.shape[0] - 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)


def displayLanes(original_image: np.array, lanes: np.array) -> np.array:
    """
    Create the image with the visible found lines
    :param original_image: The original(normal) image
    :param lanes: Found lanes
    :return: The new image, black background with coloured lanes
    """
    _lanes_image = np.zeros_like(original_image)
    if lanes is not None:
        for lane in lanes:
            x1, y1, x2, y2 = lane
            try:
                cv2.line(_lanes_image, (x1, y1), (x2, y2), (0, 255, 0), 10)
                #  displayText(_lanes_image, str(x1))
            except OverflowError:
                print(f'Overflow when drawing line!, coords: x1={x1}, y1={y1}, x2={x2}, y2={y2}')

    return _lanes_image


def _resizeWithAspectRatio(original_image, width=None, height=None, inter=cv2.INTER_AREA):
    # Source: https://stackoverflow.com/questions/35180764/opencv-python-image-too-big-to-display
    dim = None
    (h, w) = original_image.shape[:2]

    if width is None and height is None:
        return original_image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(original_image, dim, interpolation=inter)


def _cleanup(cap: cv2.VideoCapture = None) -> None:
    if cap != None:
        cap.release()
    cv2.destroyAllWindows()


def _generateGradients(original_image: np.array) -> np.array:
    """
    Apply the Canny algorithm to the given image
    :param original_image: Original image
    :return: The gradients image
    """
    grayscale_image = cv2.cvtColor(original_image, cv2.COLOR_RGB2GRAY)  # Convert to grayscale
    # Apply Gaussian Blur to grayscale image to smooth out edges (optional, since cv2.Canny can also apply it)
    blurred_image = cv2.GaussianBlur(grayscale_image, (5, 5), 0)
    return cv2.Canny(blurred_image, 50, 150)


def _regionOfInterest(original_image: np.array) -> np.array:
    """
    Apply a mask to the image, and return the image after it was masked.
    :param original_image: Original image
    :return: The image with the mask applied to it.
    """
    image_height = original_image.shape[0]  # 0 => Height | 1 => Width
    # Trying different values:
    polygons = np.array([
        [(0, image_height), (1270, image_height - 200), (550, 250)]  # Triangle, that in this case covers the road
    ])
    # Values that work:
    #
    # polygons = np.array([
    #     [(200, image_height), (1100, image_height - 200), (550, 250)]  # Triangle, that in this case covers the road
    # ])
    #
    mask = np.zeros_like(original_image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(original_image, mask)
    return masked_image


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


def _averageSlopeIntercept(original_image: np.array, lanes: np.array) -> np.array:
    """
    Average the lanes to get a smoother line
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

    left_lane = _getCoordinates(original_image, left_fit_average)
    right_lane = _getCoordinates(original_image, right_fit_average)

    return np.array([left_lane, right_lane])
