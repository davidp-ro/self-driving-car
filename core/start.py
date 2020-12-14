"""
 brief: Main file for the self-driving-car project.
 author: David Pescariu | https://github.com/davidp-ro

 Running:
    python3.7 or python3 start.py

    Args:
        --headless -> Ensures that it can run headless by disabling the previews

 Config:
    GENERAL:
     greyscaleModifier -> Increase Contrast: negative value, Decrease Contrast: positive value
     oneLineTrackingThreshold -> Threshold for the distance that is kept from the lane (in single-lane following mode)

    DEBUG:
     previewHeight -> Height of the preview window, scale is maintained
     enableDebugImages -> If true, show the image with lanes on it, and all the other active ones.
     showMaskedImage -> Show masked images
     showGrayscaleImage -> Show the grayscale version of the original_image

 copyright: GNU GPL v3 License
"""

__version__ = "1.2"

import sys
from controller import Controller
from video_processor import VideoProcessor

config = {
    'greyscaleModifier': 0,
    'oneLineTrackingThreshold': 10,

    'previewHeight': 240,
    'enableDebugImages': True,
    'showMaskedImage': True,
    'showGrayscaleImage': False,
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

    try:
        processor.run()
    except KeyboardInterrupt:
        print('==[Stopping]==')

    # Finished / Stopped running (pressed q / ctrl-c / Exception was thrown)
    controller.stop()


if __name__ == '__main__':
    main()
