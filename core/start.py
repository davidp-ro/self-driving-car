"""
 brief: Main file for the self-driving-car project.
 author: David Pescariu | https://github.com/davidp-ro

 Running:
    python3.7 or python3 start.py

    Args:
        --headless -> Ensures that it can run headless by disabling the previews

 Config:
    GENERAL:
     greyscaleModifier -> Increase Contrast: slightly: -75, significantly: -150, or lighten/decrease: positive_value
     oneLineTrackingThreshold -> Threshold for the distance that is kept from the lane (in single-lane following mode)

    DEBUG:
     previewHeight -> Height of the preview window, scale is maintained
     enableDebugImages -> If true, show the image with lanes on it, and all the other active ones.
     showMaskedImage -> Show masked images
     showGrayscaleImage -> Show the grayscale version of the original_image

 copyright: GNU GPL v3 License
"""

__version__ = "1.1"

import sys
from controller import Controller
from video_processor import VideoProcessor

config = {
    'greyscaleModifier': -50,
    'oneLineTrackingThreshold': 10,

    'previewHeight': 480,
    'enableDebugImages': True,
    'showMaskedImage': False,
    'showGrayscaleImage': False,
    'showConfigText': False,
}


def main():
    if '--headless' in sys.argv:
        # Make sure the config is valid for running headless
        config['enableDebugImages'] = False
        config['showPreviewWindow'] = False
        config['showMaskedImage'] = False
        config['showGrayscaleImage'] = False

    controller = Controller('/dev/ttyACM0', baudrate=9600)
    processor = VideoProcessor(config, controller)
    processor.run()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('==[Stopping]==')
