from controller import Controller

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


def main():
    from time import sleep

    controller = Controller('/dev/ttyACM0', baudrate=9600)
    controller.goForward()
    sleep(2)
    controller.stop()
    controller.reverse()
    sleep(2)
    controller.stop()
    return None


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('-----------')
