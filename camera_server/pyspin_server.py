#####################################################################
#                                                                   #
# PySpin_camera_server.py                                       #
#                                                                   #
# This file is part of the labscript suite (see                     #
# http://labscriptsuite.org) and is licensed under the Simplified   #
# BSD License. See the license.txt file in the root of the project  #
# for the full license.                                             #
#                                                                   #
# This camera server is an extension of camera_server.py.           #
# PySpin_camera_server implements a server and BLACS            #
# cameras using FLIR's python wrapper for FlyCapture2.              #
#                                                                   #
# To start the server:                                              #
# Run a command line in the directory containing this file          #
# and type:                                                         #
# python PySpin_camera_server.py <cameraNameFromBlacs>          #
# or type:                                                          #
# python PySpin_camera_server <cameraNameFromBlacs>             #
# <width> <height> <offsetX> <offsetY>                              #
# The optional inputs <width>, <height>, <offsetX>, and             #
# <offsetY> define an acquistion ROI for the camera.                #
# The default ROI is the full sensor.                               #
# Note that the acquisition ROI is overwritten on a per-shot        #
# basis if your BLACS camera's acquisition_ROI property is          #
# not None.                                                         #
#                                                                   #
# Author: dsbarker                                                  #
#                                                                   #
#####################################################################

# Future imports to ease change to Python 3.
# Only print_function is strictly necessary,
# but importing all to prevent issues with
# potential future features.
from __future__ import print_function, absolute_import, unicode_literals, division
from labscript_utils import check_version, PY2
if PY2:
    import Queue
else:
    import queue as Queue
import PySpin
import numpy as np
import zprocess
import sys
import time
import logging

import labscript_utils.h5_lock
import h5py

import labscript_utils.shared_drive
# importing this wraps zlock calls around HDF file openings and closings:
from labscript_utils.camera_server import CameraServer
from labscript_utils.connections import _ensure_str

check_version('zprocess', '1.3.3', '3.0')

# Set up some basic logging:
logging.basicConfig(format='%(asctime)s %(levelname)s %(name)s: %(message)s',
                    level=logging.DEBUG)
logger = logging.getLogger('pyspin_server')
f_handler = logging.FileHandler('pyspin_server.log')
formatter = logging.Formatter('%(asctime)s %(levelname)s %(name)s: %(message)s')
f_handler.setFormatter(formatter)
f_handler.setLevel(logging.DEBUG)
logging.getLogger().addHandler(f_handler)

"""
PySpin conatins a camera class, but it is based on C/C++ code and
thus it is a little bulky to work with.  So here we will define a few small
helper methods to set components that we need to control.  We will define
several:
1. set_property
2. get_property
3. setup_camera
4. set_ROI
5. acquire_multiple_images
6. print_device_info
"""
def set_property(property, value):
    if property.GetAccessMode() == PySpin.NI:
        logger.warning('Property not implemented.')
    elif property.GetAccessMode() == PySpin.NA:
        logger.warning('Property {0:s} not avialable.'.format(property.GetName()))
    elif property.GetAccessMode() == PySpin.RO:
        logger.warning('Property {0:s} read only.'.format(property.GetName()))
    else:
        property.SetValue(value)


def get_property(property):
    val = None
    if property.GetAccessMode() == PySpin.NI:
        logger.warning('Property not implemented.')
    elif property.GetAccessMode() == PySpin.NA:
        logger.warning('Property {0:s} not avialable.'.format(property.GetName()))
    else:
        val = property.GetValue()

    return val


def setup_camera(cam, slope='rising'):
    """
    This function turns off certain camera features like gain, gamma (if available),
    sharpness (if available).  It also sets
    """
    try:
        # Set the exposure mode to trigger width.
        logger.debug('Set the exposure mode to trigger width ...')
        set_property(cam.ExposureMode, PySpin.ExposureMode_TriggerWidth )

        # Turn off auto gain and set gain to zero:
        logger.debug('Setting camera gain to 0 dB ...')
        set_property(cam.GainAuto, PySpin.GainAuto_Off)
        set_property(cam.Gain, 0)

        # Can we auto black level, if so turn it off:
        if cam.BlackLevelAuto.GetAccessMode() > PySpin.NA:
            logger.debug('Setting auto-black level to off ...')
            set_property(cam.BlackLevelAuto, PySpin.BlackLevelAuto_Off)

        # Otherwise, set the black level to zero:
        logger.debug('Setting black level to 0.0 ...')
        set_property(cam.BlackLevel, 0.0)

        # Turn off gamma, if implemented and availabe:
        if cam.GammaEnable.GetAccessMode() > PySpin.NA: #Not Impelemtned
            logger.info('Turning off gamma adjust ...')
            set_property(cam.GammaEnable, False)

        # Turn off sharpness, if implemented and availabe:
        if cam.SharpeningEnable.GetAccessMode() > PySpin.NA: #Not Impelemtned
            logger.info('Turning off sharpening ...')
            set_property(cam.SharpeningEnable, False)

        # Ensure trigger mode off
        # The trigger must be disabled in order to configure whether the source
        # is software or hardware.
        logger.debug('Setting TriggerMode to off in order to configure the trigger ...')
        set_property(cam.TriggerMode, PySpin.TriggerMode_Off)

        # Select trigger source
        # The trigger source must be set to hardware or software while trigger
        # mode is off.
        logger.debug('Setting TriggerSource to Line0 ...')
        set_property(cam.TriggerSource, PySpin.TriggerSource_Line0)

        # Select trigger rising/falling edge:
        if slope == 'rising':
            logger.debug('Setting Trigger Activation to Level High ...')
            set_property(cam.TriggerActivation, PySpin.TriggerActivation_RisingEdge)
        elif slope == 'falling':
            logger.debug('Setting Trigger Activation to Level Low ...')
            set_property(cam.TriggerActivation, PySpin.TriggerActivation_FallingEdge)
        else:
            logger.warning('Did not understand slope type \'%s\', defaulting to rising.' % slope)
            logger.debug('Setting Trigger Activation to Level High ...')
            set_property(cam.TriggerActivation, PySpin.TriggerActivation_RisingEdge)

        # Turn trigger mode on
        # Once the appropriate trigger source has been set, turn trigger mode
        # on in order to retrieve images using the trigger.
        logger.debug('Turning trigger mode back on ...')
        set_property(cam.TriggerMode, PySpin.TriggerMode_On)

        # Set the acquisition mode to Multi Frame:
        logger.debug('Setting Acquisition Mode to multi-frame ...')
        set_property(cam.AcquisitionMode, PySpin.AcquisitionMode_MultiFrame)

    except PySpin.SpinnakerException as ex:
        logger.error('Error: %s' % ex)


def set_ROI(cam, offsetX, offsetY, width, height):
    """
    set_ROI sets the region of intereEst by specifying:
        offsetX: X (width) offset in pixels
        offsetY: Y (height) offset in pixels
        width: width in pixels
        height: height in pixels

    There appear to be some rules for the Flea3 cameras that
    are hard-coded into this program.

    offsetX and minOffsetX must be divisible by 8
    offsetY and minOffsetY must be divisible by 2
    width and minWidth must be divisible by 16
    height and minHeight must be divisible by 2
    """
    try:
        logger.debug('Setting width ...')
        if (width - cam.Width.GetMin()) % 16 != 0:
            width = 16*int(np.ceil( (width - cam.Width.GetMin())/16. )) +\
                cam.Width.GetMin()
            logger.warning('Setting width to %d to match rules!' % width)
        if width > cam.Height.GetMax() or width < cam.Height.GetMin():
            logger.error('Width %d either too large or too small!' % width)
        else:
            set_property(cam.Width, width)

        logger.debug('Setting height ...')
        if (height - cam.Height.GetMin()) % 2 != 0:
            height = 2*int(np.ceil( (height - cam.Height.GetMin())/2. )) +\
                cam.Height.GetMin()
            logger.warning('Setting height to %d to match rules!' % height)

        if height > cam.Height.GetMax() or height < cam.Height.GetMin():
            logger.error('Height %d either too large or too small!' % height)
        else:
            set_property(cam.Height, height)

        logger.debug('Setting offsetX...')
        if (offsetX - cam.OffsetX.GetMin()) % 8 != 0:
            offsetX = 8*int(np.round( (offsetX - cam.OffsetX.GetMin())/8. )) +\
                cam.OffsetX.GetMin()
            logger.warning('Setting X offset to %d to match rules!' % offsetX)
        if offsetX > cam.OffsetX.GetMax() or offsetX < cam.OffsetX.GetMin():
            logger.error('X offset %d either too large or too small!' % offsetX)
        else:
            set_property(cam.OffsetX, offsetX)

        logger.debug('Setting offsetY...')
        if (offsetY - cam.OffsetY.GetMin()) % 8 != 0:
            offsetY = 2*int(np.round( (offsetY - cam.OffsetY.GetMin())/2. )) +\
                cam.OffsetY.GetMin()
            logger.warning('Setting X offset to %d to match rules!' % offsetX)
        if offsetY > cam.OffsetY.GetMax() or offsetY < cam.OffsetY.GetMin():
            logger.error('X offset %d either too large or too small!' % offsetX)
        else:
            set_property(cam.OffsetY, offsetY)

        offsetX = get_property(cam.OffsetX)
        offsetY = get_property(cam.OffsetY)
        width = get_property(cam.Width)
        height = get_property(cam.Height)

        logger.info('Image dimensions ({0:d}, {1:d}), offset ({2:d}, {3:d})'.format(
                     width, height, offsetX, offsetY))
    except PySpin.SpinnakerException as ex:
        logger.error('Error: %s' % ex)


def acquire_multiple_images(cam, n_images, timeout=PySpin.EVENT_TIMEOUT_INFINITE,
                            comm_queue=Queue.Queue()):
    # Set the number of acqueistion images:
    set_property(cam.AcquisitionFrameCount, n_images)

    width = get_property(cam.Width)
    height =  get_property(cam.Height)

    cam.BeginAcquisition()
    logger.info('Capture started for %d images.' % n_images)

    imgs = np.zeros((n_images, height, width))

    try:
        ii=0
        while ii < n_images and comm_queue.empty():
            # Retrieve next received image
            image_result = cam.GetNextImage(timeout)

            #  Ensure image completion
            if image_result.IsIncomplete():
                logger.warning('Image incomplete with image status %d ...' % image_result.GetImageStatus())
                break
            else:
                imgs[ii] = np.array(image_result.GetData(), dtype='uint8').reshape(height, width)
                if ii % 10 == 0 and n_images>15:
                    logger.info('retrieved {i}th image of {j} images'.format(i=ii+1,j=n_images))
                else:
                    logger.info('retrieved {i}th image of {j} images'.format(i=ii+1,j=n_images))
                #  Release image
                image_result.Release()
                ii+=1

        logger.info('Successfully retrieved %d images.' % d)
    except PySpin.SpinnakerException as ex:
        # We capture some sort of a Spinnaker exception here, which we print
        # for the record and move on with life.  If we raise the error, we'd
        # to restart the camera server.
        logger.error('Error: %s.  Returning empty images.' % ex)

        # If we have caught an SpinnakerException, it has been logged.  We shuold
        # then clear the images so nothing is returned.
        imgs = []
    finally:
        # End acquisition no matter what
        cam.EndAcquisition()

        return imgs


def print_device_info(cam):
    """
    This function prints the device information of the camera from the transport
    layer; please see NodeMapInfo example for more in-depth comments on printing
    device information from the nodemap.

    :param nodemap: Transport layer device nodemap.
    :type nodemap: INodeMap
    :returns: True if successful, False otherwise.
    :rtype: bool
    """
    nodemap = cam.GetTLDeviceNodeMap()

    logger.debug('   *** DEVICE INFORMATION ***   ')

    try:
        result = True
        node_device_information = PySpin.CCategoryPtr(nodemap.GetNode('DeviceInformation'))

        if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
            features = node_device_information.GetFeatures()
            for feature in features:
                node_feature = PySpin.CValuePtr(feature)
                logger.debug('%s: %s' % (node_feature.GetName(),
                                  node_feature.ToString() if PySpin.IsReadable(node_feature) else 'Node not readable'))

        else:
            logger.debug('Device control information not available.')

    except PySpin.SpinnakerException as ex:
        logger.error('Error: %s' % ex)
        return False

    logger.debug('   *** END DEVICE INFORMATION ***   ')

    return result


class PySpin_CameraServer(zprocess.ZMQServer):
    """
    The PySpin Camera Server class, handles the commands and data coming to/
    from BLACS/labscript and going from/to the camera

    Initialization arguments:
        port: server port to run on
        camera_name: the name of the camera in the connection table
        command_queue: the queue pf command objects
        results_queue: results of the acquisition
    """
    def __init__(self, port, camera_name, command_queue, results_queue):
        zprocess.ZMQServer.__init__(self, port, type='string')
        self._h5_filepath = None
        self.camera_name = camera_name
        self.command_queue = command_queue
        self.results_queue = results_queue
        self.imgs = []
        self.logger = logging.getLogger('pyspin_server.server')
        self.logger.info('Camera server started on port %d.' % port)

    def handler(self, request_data):
        """
        This handler is copied directly from the CameraServer prototype,
        but changed only that it prints the data request in the log.
        """
        try:
            self.logger.info('Received request %s.' % request_data)
            if request_data == 'hello':
                return 'hello'
            elif request_data.endswith('.h5'):
                self._h5_filepath = labscript_utils.shared_drive.path_to_local(request_data)
                self.send('ok')
                self.recv()
                self.transition_to_buffered(self._h5_filepath)
                return 'done'
            elif request_data == 'done':
                self.send('ok')
                self.recv()
                self.transition_to_static(self._h5_filepath)
                self._h5_filepath = None
                return 'done'
            elif request_data == 'abort':
                self.abort()
                self._h5_filepath = None
                return 'done'
            else:
                raise ValueError('invalid request: %s'%request_data)
        except Exception:
            if self._h5_filepath is not None and request_data != 'abort':
                try:
                    self.abort()
                except Exception as e:
                    sys.stderr.write('Exception in self.abort() while handling another exception:\n{}\n'.format(str(e)))
            self._h5_filepath = None
            raise

    def transition_to_buffered(self, h5_filepath):
        # How many images to get
        with h5py.File(h5_filepath) as f:
            groupname = self.camera_name
            group = f['devices'][groupname]
            props = labscript_utils.properties.get(f, camera_name,'device_properties')
            if not 'EXPOSURES' in group:
                print('no camera exposures in this shot.')
                return
            n_images = len(group['EXPOSURES'])
            # Find max time between images:
            if n_images > 1:
                exp_times = f['devices'][groupname]['EXPOSURES']['time']
                max_wait = 1.5*max(abs(x - y) for (x, y) in zip(exp_times[1:], exp_times[:-1]))
            else:
                max_wait = 0.1
            # Use acquisition_ROI property to set camera ROI:
            if 'acquisition_ROI' in props:
                if props['acquisition_ROI'] is not None:
                    width = props['acquisition_ROI'][0]
                    height = props['acquisition_ROI'][1]
                    offX = props['acquisition_ROI'][2]
                    offY = props['acquisition_ROI'][3]
                    print('resetting acquisition ROI')
                    # Tell acquisition mainloop to reset ROI:
                    self.command_queue.put(['set_ROI', (width, height, offX, offY)])
        self.logger.info('Configured for %d images and max_wait = %f s.' %
                         (n_images, max_wait))
        # Tell the acquisition mainloop to get some images:
        self.command_queue.put(['acquire', (n_images, int(1500*max_wait))])

    def transition_to_static(self, h5_filepath):
        start_time = time.time()
        with h5py.File(h5_filepath) as f:
            groupname = self.camera_name
            group = f['devices'][groupname]
            if not 'EXPOSURES' in group:
                self.logger.warning('no camera exposures in this shot.')
                return
            n_images = len(group['EXPOSURES'])
            # For some reason, the h5 file is being read as bytes and not a string:
            img_type = f['devices'][groupname]['EXPOSURES']['frametype']
            img_set = list(set(img_type))
            try:
                images = self.results_queue.get(timeout=1)
            except Queue.Empty:
                self.logger.error('There was some problem in the acquisition!')
                images = []

            # The number of images that comes back must be equal
            if len(images) != n_images:
                raise ValueError('Did not capture all the images expected!')

            # Save images only if there
            n_acq = len(images)
            self.logger.info('Saving {a} images.'.format(a=n_acq))
            # Create the group in which we will save the images:
            group = f.create_group('/images/' + f['devices'][groupname].attrs.get('orientation') + '/' + groupname)
            # Save images:
            imgs_toSave = {}
            for f_type in img_set:
                imgs_toSave[f_type] = []
                idx = 0
                # all images should be same size:
                for idx in range(n_acq):
                    if img_type[idx] == f_type:
                        imgs_toSave[f_type].append(images[idx])
                # print('Creating dataset.')
                group.create_dataset(f_type,data=np.array(imgs_toSave[f_type]))
                """images_to_save = [imgs[idx] if img_type[idx] == f_type for idx in range(n_acq)]
                group.create_dataset(f_type,data=np.array(images_to_save))"""
                self.logger.info(
                    _ensure_str(f_type) + ' camera shots saving time: ' + \
                    '{0:.6f}'.format(time.time() - start_time)+ 's')

    def abort(self):
        # If abort gets called, probably need to break out of grabMultiple.
        # Put a dummy command into the queue to accomplish that.
        self.command_queue.put(['abort', None])



def acquisition_mainloop(command_queue, results_queue, camera, h5_attrs):
    """
    The acquisition main loop, which runs as a seperate process.  It takes
    several arguments:
        command_queue: the commands coming into the acquisition loop
        results_queue: the results coming out of the acquisition loop
        camera: the PySpin camera object for the camera itself.
        h5_attrs: attributes of the camera in question.
    """
    while True:
        command, args = command_queue.get()
        #print(command, args)
        if command == 'acquire':
            (n_images, timeout) = args
            result = acquire_multiple_images(camera, n_images, timeout)
        elif command == 'set_ROI':
            # Only perform ROI reset when necessary.
            if (width,height,offX,offY) != args:
                width = args[0]
                height = args[1]
                offX = args[2]
                offY = args[3]
                pyspin_utils.set_ROI(cam, width, height, offX, offY)
            continue # skip put into results_queue
        elif command == 'abort':
            # command to cause grabMultiple to break
            try:
                results_queue.get(timeout=1) # Pull bad images out of queue.
            except Queue.Empty:
                logger.info('Tried to pull bad images from queue, none present.')
            continue # skip put into results_queue
        elif command == 'quit':
            break
        else:
            result = Exception('invalid command')
        results_queue.put(result)



if __name__ == '__main__':
    """
    To start the server, type: "python pyspin_camera_server <camera_name>".

    Alternatively, one can start the server by python pyspin_camera_server
    <camera_name> offsetX offsetY width height to specify an initial ROI.
    """
    import threading
    from labscript_utils.labconfig import LabConfig
    import labscript_utils.properties
    import h5py

    lc = LabConfig()

    logger_main = logging.getLogger('pyspin_server.main')

    try:
        camera_name = sys.argv[1]
    except IndexError:
        logger.error('Call me with the name of a camera as defined in BLACS.')
        sys.exit(0)

    # Get the h5 path and camera properties.
    logger_main.debug('Getting connection table.')
    h5_filepath = lc.get('paths', 'connection_table_h5')
    with h5py.File(h5_filepath) as f:
        h5_attrs = labscript_utils.properties.get(f, camera_name,
                                                   'device_properties')
        h5_conn = labscript_utils.properties.get(f, camera_name,
                                                   'connection_table_properties')


    # Define synchronization Queues:
    command_queue = Queue.Queue()
    results_queue = Queue.Queue()

    # Retrieve singleton reference to system object
    logger_main.debug('Getting camera system instance ...')
    system = PySpin.System.GetInstance()

    # Retrieve list of cameras from the system
    logger_main.debug('Searching for cameras...')
    camList = system.GetCameras()
    numCams = camList.GetSize()
    logger_main.info('Found %d cameras' % numCams)

    if numCams > 0:
        # Get the camera object of interest:
        try:
            logger_main.info('Initializing camera %d.' % h5_attrs['serial_number'])
            cam = camList.GetBySerial(str(h5_attrs['serial_number']))
            cam.Init()
            print_device_info(cam)
            setup_camera(cam, slope=h5_attrs['trigger_edge_type'])

            # Start the camera server:
            server = PySpin_CameraServer(h5_conn['BIAS_port'], camera_name,
                                         command_queue, results_queue)

            # Start the acquisition loop for the camera of interest:
            acquisition_thread = threading.Thread(
                target=acquisition_mainloop,
                args=(command_queue, results_queue, cam, h5_attrs)
                )
            acquisition_thread.daemon=True
            acquisition_thread.start()

            if len(sys.argv)-2 == 4:
                 command_queue.put([set_ROI,
                                    int(sys.argv[2]), int(sys.argv[3]),
                                    int(sys.argv[4]), int(sys.argv[5])
                                   ])

            server.shutdown_on_interrupt()
            command_queue.put(['quit', None])

            # The join should timeout so that infinite grab loops do not persist.
            acquisition_thread.join(1.0)
        finally:
            logger_main.debug('Deinitializing camera ...')
            cam.DeInit()
            del cam

            # Clear camera list before releasing system
            logger_main.debug('Clearing camera instance...')
            camList.Clear()

            # Release system instance
            logger_main.debug('Releasing system instance...')
            system.ReleaseInstance()
    else:
        logger_main.error('No cameras detected!')

        # Clear camera list before releasing system
        logger_main.debug('Clearing camera instance...')
        camList.Clear()

        # Release system instance
        logger_main.debug('Releasing system instance...')
        system.ReleaseInstance()
