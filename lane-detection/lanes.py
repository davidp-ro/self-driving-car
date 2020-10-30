from detection import Detect

config = {
    'showPreviewWindow': True,
    'previewHeight': 480,

    'detectOuterLanes': False,
    'ignoreRightOuterLine': True,
    'drawOuterLanes': False,

    'showMaskedImage': False,
    'showGrayscaleImage': False,
    'showConfigText': True,

    'carPositionDetection': True,
    'oneLineTrackingThreshold': 10,
}

detection = Detect(config)

# FILE_NAME = "test_data/test_image.jpg"
FILE_NAME = "test_data/test_video.mp4"

# detection.detect(FILE_NAME, "image")
detection.detect(FILE_NAME, "video")

"""
About:
    Part of my @davidp-ro project to build a (indoors-sized) self driving car
    
    Initially followed https://www.youtube.com/watch?v=eLTLtUVuuy4&list=LL&index=8
    
Repository: https://github.com/davidp-ro/self-driving-car
License: GNU GPL-3.0
"""
