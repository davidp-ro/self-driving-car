import cv2
import numpy as np
import sys
import warnings

"""
Uses the Hough Lines Method to detect lanes.

 Atm tested with the video in 'test_data/test_video.mp4', and lane regions optimized
for a video that is 720x1280

Config:
    previewHeight -> Height of the preview window, scale is maintained
    detectOuterLanes -> If false will skip detecting outer lanes
    ignoreRightOuterLine -> If true, ignore the right outer lane to avoid overlapping
    drawOuterLanes -> Show outer lanes (if detectOuterLanes is True)
    showMaskedImage -> Show masked images
    showGrayscaleImage -> Show the grayscale version of the original_image
    showConfigText -> Show the small config 'dump' on screen
    
Call:
    from detection import detect
    detect(
        file_name -> Path of the video/image
        data_type -> "video" or "image"
    )
"""

config = {
    'previewHeight': 480,
    'detectOuterLanes': True,
    'ignoreRightOuterLine': True,
    'drawOuterLanes': True,
    'showMaskedImage': False,
    'showGrayscaleImage': False,
    'showConfigText': True
}


def detect(file_name: str, data_type: str) -> None:
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


def analyzeFrame(current_frame: np.array, file_name: str = "Unknown filename") -> None:
    """
    Analyze a given frame

    :param current_frame: The current frame
    :param file_name: Optional, File name to be displayed on the title bar
    """
    canny = _generateGradients(current_frame)

    # Inner lanes:
    cropped_image = _regionOfInterest(canny)
    if config['showMaskedImage']:
        cv2.imshow("Masked Inner Lanes", _resizeWithAspectRatio(cropped_image, height=config['previewHeight']))
    found_lanes = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=5)
    averaged_inner_lines = _averageSlopeIntercept(current_frame, found_lanes)

    # Outer lanes:
    if config['detectOuterLanes']:
        cropped_image = _regionOfInterest(canny, whole_road=True)
        if config['showMaskedImage']:
            cv2.imshow("Masked Outer Lanes", _resizeWithAspectRatio(cropped_image, height=config['previewHeight']))
        found_lanes = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=5)
        averaged_outer_lines = _averageSlopeIntercept(current_frame, found_lanes)
    else:
        averaged_outer_lines = None

    lanes_image = displayLanes(
        current_frame,
        inner_lanes=averaged_inner_lines,
        outer_lanes=averaged_outer_lines if config['drawOuterLanes'] else None
    )
    final_image = cv2.addWeighted(current_frame, 0.8, lanes_image, 1, 1)

    final_image = _resizeWithAspectRatio(final_image, height=config['previewHeight'])

    if config['showConfigText']:
        displayConfig(final_image)
    cv2.imshow(f'Lane Detection - {file_name} | Press Q to Quit', final_image)


def displayConfig(original_image: np.array) -> None:
    """
    Display the config 'dump' on screen
    :param original_image: The original(normal) image
    """
    message = f"Config: detectOuter={config['detectOuterLanes']}&draw={config['drawOuterLanes']} ignoreRightOuter={config['ignoreRightOuterLine']}"
    cv2.putText(original_image, message,
                (5, original_image.shape[0] - 5),
                cv2.FONT_HERSHEY_DUPLEX, 0.6,
                (255, 255, 255), 2)


def displayLanes(original_image: np.array, inner_lanes: np.array, outer_lanes: np.array) -> np.array:
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
            if config['ignoreRightOuterLine'] and x1 > int(original_image.shape[1] / 2):
                continue
            try:
                cv2.line(lanes_image, (x1, y1), (x2, y2), (0, 0, 255), 5)
            except OverflowError:
                print(f'Overflow when drawing OUTER lane!, coords: x1={x1}, y1={y1}, x2={x2}, y2={y2}')

    return lanes_image


def _resizeWithAspectRatio(original_image: np.array, width: int = None, height: int = None) -> np.array:
    """
    Resize a image, while keeping it's aspect ratio. Give either a width or a height

    Source: https://stackoverflow.com/questions/35180764/opencv-python-image-too-big-to-display

    :param original_image: The original(normal) image
    :param width: Optional, the new width
    :param height: Optional, the new height
    :return: The resized image
    """
    dim = None
    inter = cv2.INTER_AREA
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
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


def _generateGradients(original_image: np.array) -> np.array:
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
    if config['showGrayscaleImage']:
        cv2.imshow("Greyscale", _resizeWithAspectRatio(blurred_image, height=config['previewHeight']))
    return cv2.Canny(blurred_image, 50, 150)


def _regionOfInterest(original_image: np.array, whole_road: bool = False) -> np.array:
    """
    Apply a mask to the image, and return the image after it was masked.

    :param original_image: Original image
    :param whole_road: If true returns the regions for the outer lines, if false, returns the inner ones
    :return: The image with the mask applied to it.
    """
    if not whole_road:
        return __regionOfInterestForLane(original_image)
    else:
        return __regionsOfInterestForRoad(original_image)


def __regionOfInterestForLane(original_image: np.array) -> np.array:
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


def __regionsOfInterestForRoad(original_image: np.array) -> np.array:
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
        [(0, image_height - 100), (300, image_height - 200), (300, midway - 100), (0, midway + 100)],  # LEFT
        [(800, image_height), (1200, image_height), (1000, midway), (700, midway)]  # RIGHT
    ])
    mask = np.zeros_like(original_image)
    cv2.fillPoly(mask, polygons, 255)
    # cv2.imshow("MASK FOR ROAD", mask)
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

    left_lane = _getCoordinates(original_image, left_fit_average)
    right_lane = _getCoordinates(original_image, right_fit_average)

    return np.array([left_lane, right_lane])
