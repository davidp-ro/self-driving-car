"""
 brief: Part of the self-driving-car project, different small utility functions.
 author: David Pescariu | https://github.com/davidp-ro

 copyright: GNU GPL v3 License
"""

__version__ = '1.0'

import numpy as np
import cv2


class Utils:
    @staticmethod
    def resizeWithAspectRatio(original_image: np.array, width: int = None, height: int = None) -> np.array:
        """
        Resize a image, while keeping it's aspect ratio. Give either a width or a height

        Source: https://stackoverflow.com/questions/35180764/opencv-python-image-too-big-to-display

        :param original_image: The original(normal) image
        :param width: Optional, the new width
        :param height: Optional, the new height
        :return: The resized image
        """
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
