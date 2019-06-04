import PySpin
import numpy as np
import logging
import sys

from labscript_utils.camera_server.pyspin_server import set_property, setup_camera, set_ROI, acquire_multiple_images, print_device_info

if __name__ == '__main__':
    # Make a sub logger:
    logger = logging.getLogger('psypin_server.test_camera')
    logger.info('Testing camera; please provide hardware tiggers...')

    # Retrieve singleton reference to system object
    logger.debug('Getting system instance...')
    system = PySpin.System.GetInstance()

    # Retrieve list of cameras from the system
    logger.debug('Searching for cameras...')
    camList = system.GetCameras()
    numCams = camList.GetSize()
    logger.info('Found %d cameras' % numCams)

    if numCams > 0:
        # Get the camera object of interest:
        cam = camList.GetBySerial(sys.argv[1])
        print_device_info(cam)

        cam.Init()

        setup_camera(cam)
        """if len(sys.argv)-2 == 4:
            set_ROI(cam, int(sys.argv[2]), int(sys.argv[3]),
                    int(sys.argv[4]), int(sys.argv[5]))"""

        imgs = acquire_multiple_images(cam, 3)

        logger.debug('Saving images ...')
        np.savez('images.npz', imgs=imgs)

        set_property(cam.TriggerMode, PySpin.TriggerMode_Off)
        logger.info('Deinitializing camera.')
        cam.DeInit()
        del cam
    else:
        logger.info('No cameras found!')

    # Clear camera list before releasing system
    logger.debug('Clearing camera instance...')
    camList.Clear()

    # Release system instance
    logger.debug('Releasing system instance...')
    system.ReleaseInstance()
