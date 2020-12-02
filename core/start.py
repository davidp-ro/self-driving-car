"""
 brief: Main file for the self-driving-car project.
 author: David Pescariu | https://github.com/davidp-ro

 Running:
    python3.7 or python3 start.py

    Args:
        --headless -> Ensures that it can run headless by disabling the previews

 Config:
    showPreviewWindow -> If true will show the window with the lanes drawn on it
    previewHeight -> Height of the preview window, scale is maintained
    detectOuterLanes -> If false will skip detecting outer lanes
    ignoreRightOuterLine -> If true, ignore the right outer lane to avoid overlapping
    drawOuterLanes -> Show outer lanes (if detectOuterLanes is True)
    showMaskedImage -> Show masked images
    showGrayscaleImage -> Show the grayscale version of the original_image
    showConfigText -> Show the small config 'dump' on screen
    oneLineTrackingThreshold -> Threshold for the distance that is kept from the lane (in single-lane following mode)
    greyscaleModifier -> Darken slightly: -75, Darken significantly: -150, Lighten: positive_value

 copyright: GNU GPL v3 License
"""

import sys
from controller import Controller
from video_processor import VideoProcessor

__version__ = "1.0"

config = {
    'showPreviewWindow': False,
    'previewHeight': 480,

    'detectOuterLanes': False,
    'ignoreRightOuterLine': True,
    'drawOuterLanes': False,

    'showMaskedImage': False,
    'showGrayscaleImage': False,
    'showConfigText': False,

    'oneLineTrackingThreshold': 10,
    'greyscaleModifier': -75,
}


def main():
    if '--headless' in sys.argv:
        # Make sure the config is valid for running headless
        config['showPreviewWindow'] = False
        config['drawOuterLanes'] = False
        config['showMaskedImage'] = False
        config['showGrayscaleImage'] = False
        config['showConfigText'] = False

    controller = Controller('/dev/ttyACM0', baudrate=9600)
    processor = VideoProcessor(config, controller)
    processor.run()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('==[Stopping]==')
