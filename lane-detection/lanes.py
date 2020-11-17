from detection import Detect

config = {
    'showPreviewWindow': False,
    'previewHeight': 480,

    'detectOuterLanes': False,
    'ignoreRightOuterLine': True,
    'drawOuterLanes': False,

    'showMaskedImage': False,
    'showGrayscaleImage': False,
    'showConfigText': False,

    'carPositionDetection': True,
    'oneLineTrackingThreshold': 10,

    # roadType -> small = 1 lane/director, medium / large = large roads / highways, test to see which one works better
    'roadType': 'small',
    # greyscaleModifier -> darken slightly: -75, darken significantly: -150, lighten: positive_value
    'greyscaleModifier': -75,
}

detection = Detect(config)

# === Test data ===
# FILE_NAME = "test_data/test_image.jpg"  # roadType: small
FILE_NAME = "test_data/test_video.mp4"  # roadType: small
# FILE_NAME = "test_data/test_video_2.mp4"  # roadType: medium

# detection.detect(FILE_NAME, "image")
detection.detect(FILE_NAME, "video")

"""
About:
    Part of my @davidp-ro project to build a (indoors-sized) self driving car
    
    Initially followed https://www.youtube.com/watch?v=eLTLtUVuuy4&list=LL&index=8
    
Repository: https://github.com/davidp-ro/self-driving-car
License: GNU GPL-3.0
"""
