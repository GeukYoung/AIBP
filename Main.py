"""
@author: SNUBH: 02/07/2024

"""

from __future__ import unicode_literals

import argparse
import logging
import time
from IntellivueProtocol.IntellivueDecoder import IntellivueDecoder
from IntellivueProtocol.RS232 import RS232
from IntellivueProtocol.IntellivueDistiller import IntellivueDistiller
from TelemetryStream import TelemetryStream
# from TelemetryStream import attach_loggers
from QualityOfSignal import QualityOfSignal as QoS
from collections import deque
import multiprocessing as mp
import numpy as np
import bisect
import tkinter as tk
from tkinter import Toplevel
# import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from scipy import signal
from scipy.signal import butter, filtfilt
from functools import partial
import threading
import tensorflow as tf
import serial
import os
import easydict
args = easydict.EasyDict({
    "gui": "SimpleStripchart",
})

__description__ = "AIBP"
__version_info__ = ('0', '0', '1')
__version__ = '.'.join(__version_info__)

# updated 0.7.3 accounts for 8000Hz data collection throughout

logging.disable()

# Wrapper for UCSF QoS code
def qos(*args, **kwargs):
    my_qos = QoS()
    history = kwargs.get('sampled_data')
    if history:
        res = my_qos.isPPGGoodQuality(history.get('Pleth').get('samples').y,
                                      125)  # For Philips monitors, Pleth frequency is 32 per 1.024/4 second
        return {'qos': res}
    else:
        return -1


class CriticalIOError(IOError):
    """Need to tear the socket down and reset."""
    pass


class PhilipsTelemetryStream(TelemetryStream):
    """
    This class utilizes the data structures defined in IntellivueDecoder and
    the functions to communicate with the monitor via RS232.
    """

    # def __init__(self, serialPort, patientDirectory, selectedDataTypes):
    def __init__(self, *args, **kwargs):
        super(PhilipsTelemetryStream, self).__init__(*args, **kwargs)

        self.logger.name = 'PhilipsTelemetry'

        serialPort = kwargs.get('port')
        selectedDataTypes = kwargs.get('values')[::2]  # These come in as value, freq pairs; just need names

        # self.port = serialPort by JG
        self.port = serialPort
        self.rs232 = None  # This will be the socket object

        # Initialize Intellivue Decoder and Distiller
        self.decoder = IntellivueDecoder()
        self.distiller = IntellivueDistiller()

        # Initialize variables to keep track of time, and values to collect

        # Note: The listener automatically shuts down after this many seconds
        # Max is
        self.dataCollectionTime = 72 * 60 * 60  # seconds
        self.dataCollection = {'RelativeTime': self.dataCollectionTime * 8000}
        self.KeepAliveTime = 0
        self.messageTimes = []
        self.desiredWaveParams = {'TextIdLabel': selectedDataTypes}
        self.initialTime = 0
        self.relativeInitialTime = 0

        #  Initialize Messages
        self.AssociationRequest = self.decoder.writeData('AssociationRequest')
        self.AssociationAbort = self.decoder.writeData('AssociationAbort')
        self.ConnectIndication = {}
        self.AssociationResponse = ''
        self.MDSCreateEvent = {}
        self.MDSParameters = {}
        self.MDSCreateEventResult = ''
        self.MDSSetPriorityListWave = self.decoder.writeData('MDSSetPriorityListWAVE', self.desiredWaveParams)
        self.MDSSetPriorityListNumeric = ''
        self.MDSSetPriorityListResultWave = {}
        self.MDSSetPriorityListResultNumeric = {}
        self.MDSGetPriorityList = self.decoder.writeData('MDSGetPriorityList')
        self.MDSGetPriorityListResult = {}
        self.ReleaseRequest = self.decoder.writeData('ReleaseRequest')
        self.MDSExtendedPollActionNumeric = self.decoder.writeData('MDSExtendedPollActionNUMERIC',
                                                                   self.dataCollection)
        self.MDSExtendedPollActionWave = self.decoder.writeData('MDSExtendedPollActionWAVE', self.dataCollection)
        self.MDSExtendedPollActionAlarm = self.decoder.writeData('MDSExtendedPollActionALARM', self.dataCollection)
        self.KeepAliveMessage = self.decoder.writeData('MDSSinglePollAction')

        # Boolean to keep track of whether data should still be polled
        self.data_flow = False

        self.last_read_time = time.time()
        self.timeout = 10  # Seconds to wait before reset to transient failures

        self.last_keep_alive = time.time()

    def initiate_association(self, blocking=False):

        # There are 2 phases to the association, the request/response and the creation event
        # If any phase fails, raise an error. If blocking, raising an `IOError` will wait and
        # try again, raising a `CriticalIOError` passes it up to reset the socket

        def request_association():

            if not self.rs232:
                logging.warning('Trying to send an Association Request without a socket!')
                raise CriticalIOError
            try:
                self.rs232.send(self.AssociationRequest)
                self.logger.debug('Sent Association Request...')
            except:
                self.logger.warning("Unable to send Association Request")
                raise IOError

        def receive_association_response():
            association_message = self.rs232.receive()

            # Could handle no message in getMessageType (hrm)
            if not association_message:
                logging.warning('No association received')
                raise IOError

            message_type = self.decoder.getMessageType(association_message)
            self.logger.debug('Received ' + message_type + '.')

            # If we got an AssociationResponse we can return
            if message_type == 'AssociationResponse':
                return association_message

            elif message_type == 'TimeoutError':
                # Tolerate timeouts for a while in case monitor is resetting
                raise IOError

            # Fail and reset!
            elif message_type == 'AssociationAbort' or message_type == 'ReleaseRequest' or message_type == 'Unknown':
                raise CriticalIOError

            # If data still coming in from a previous connection or no data is coming in, abort/release
            elif message_type == 'MDSExtendedPollActionResult' or message_type == 'LinkedMDSExtendedPollActionResult':
                # self.rs232.send(self.AssociationAbort)
                # self.rs232.send(self.ReleaseRequest)
                # self.close()
                raise CriticalIOError

            else:
                raise IOError

        def receive_event_creation(association_message):
            # This is the create event message now
            event_message = self.rs232.receive()

            message_type = self.decoder.getMessageType(event_message)
            logging.debug('Received ' + message_type + '.')

            # ie, we got the create event response
            if message_type == 'MDSCreateEvent':
                self.AssociationResponse = self.decoder.readData(association_message)

                logging.debug("Association response: {0}".format(self.AssociationResponse))

                self.KeepAliveTime = \
                    self.AssociationResponse['AssocRespUserData']['MDSEUserInfoStd']['supported_aprofiles'][
                        'AttributeList']['AVAType']['NOM_POLL_PROFILE_SUPPORT']['AttributeValue']['PollProfileSupport'][
                        'min_poll_period']['RelativeTime'] / 8000
                self.MDSCreateEvent, self.MDSParameters = self.decoder.readData(event_message)

                # Store the absolute time marker that everything else will reference
                self.initialTime = self.MDSCreateEvent['MDSCreateInfo']['MDSAttributeList']['AttributeList']['AVAType'][
                    'NOM_ATTR_TIME_ABS']['AttributeValue']['AbsoluteTime']
                self.relativeInitialTime = \
                    self.MDSCreateEvent['MDSCreateInfo']['MDSAttributeList']['AttributeList']['AVAType'][
                        'NOM_ATTR_TIME_REL']['AttributeValue']['RelativeTime']
                if 'saveInitialTime' in dir(self.distiller):
                    self.distiller.saveInitialTime(self.initialTime, self.relativeInitialTime)

                # Send MDS Create Event Result
                self.MDSCreateEventResult = self.decoder.writeData('MDSCreateEventResult', self.MDSParameters)
                self.rs232.send(self.MDSCreateEventResult)
                logging.debug('Sent MDS Create Event Result...')
                return
            else:
                # We didn't get a properly formed create event message!
                self.logger.error('Bad handshake!')
                raise CriticalIOError

        # Keep trying until success
        if blocking:
            io_errors = 0
            while 1:
                try:
                    request_association()
                    m = receive_association_response()
                    receive_event_creation(m)
                    break
                except CriticalIOError:
                    logging.error('Critical IOError, resetting socket')
                    raise
                except IOError:
                    # Willing to tolerate 12 errors before passing it up
                    io_errors += 1
                    if io_errors >= 12:
                        logging.error('Escalating IOError, resetting socket')
                        raise
                    else:
                        logging.error('IOError, waiting to try again {0}'.format(io_errors))
                        time.sleep(2.0)
                        continue

        else:
            request_association()
            m = receive_association_response()
            receive_event_creation(m)

    # Set Priority Lists (ie what data should be polled)
    def set_priority_lists(self):
        """
        Sends MDSSetPriorityListWave
        Receives the confirmation
        """
        # Writes priority lists
        self.MDSSetPriorityListWave = self.decoder.writeData('MDSSetPriorityListWAVE', self.desiredWaveParams)

        # Send priority lists
        self.rs232.send(self.MDSSetPriorityListWave)
        logging.debug('Sent MDS Set Priority List Wave...')

        # Read in confirmation of changes
        no_confirmation = True
        while no_confirmation:

            message = self.rs232.receive()
            if not message:
                logging.warning('No priority list msg received!')
                break

            message_type = self.decoder.getMessageType(message)

            # If Priority List Result, store message, advance script
            if message_type == 'MDSSetPriorityListResult':
                PriorityListResult = self.decoder.readData(message)

                # If there are wave data objects, create a group for them
                if 'NOM_ATTR_POLL_RTSA_PRIO_LIST' in PriorityListResult['SetResult']['AttributeList']['AVAType']:
                    self.MDSSetPriorityListResultWave = PriorityListResult
                    logging.debug('Received MDS Set Priority List Result Wave.')

                no_confirmation = False

            # If MDSCreateEvent, then state failure to confirm
            elif message_type == 'MDSCreateEvent':
                no_confirmation = False
                logging.warning('Failed to confirm priority list setting.')

    def submit_keep_alive(self):
        self.rs232.send(self.KeepAliveMessage)
        self.last_keep_alive = time.time()
        logging.debug('Sent Keep Alive Message...')

    # # Extended retrieve data from monitor; this is unused but preserved from original code
    # def extended_poll(self):
    #     """
    #     Sends Extended Poll Requests for Numeric and Wave Data
    #     """
    #
    #     # Need to poll numerics to keep machine alive, but don't save if not
    #     # specified
    #     self.rs232.send(self.MDSExtendedPollActionNumeric)
    #     self.rs232.send(self.MDSExtendedPollActionWave)
    #     self.rs232.send(self.MDSExtendedPollActionAlarm)
    #     logging.info('Sent MDS Extended Poll Action for Numerics...')
    #     logging.info('Sent MDS Extended Poll Action for Waves...')
    #     logging.info('Sent MDS Extended Poll Action for Alarms...')
    #
    #     keep_alive_messages = 1
    #     self.data_flow = True
    #     while (self.data_flow):
    #
    #         message = self.rs232.receive()
    #         if not message:
    #             logging.warn('No data msg received!')
    #             self.data_flow = False
    #             break
    #
    #         message_type = self.decoder.getMessageType(message)
    #
    #         if message_type == 'AssociationAbort':
    #             logging.info('Data Collection Terminated.')
    #             self.rs232.close()
    #             self.data_flow = False
    #
    #         elif message_type == 'RemoteOperationError':
    #             logging.error('Error Message')
    #
    #         elif message_type == 'MDSSinglePollActionResult':
    #             # logging.info('Message Kept Alive!')
    #             pass
    #
    #         elif message_type == 'MDSExtendedPollActionResult' or message_type == 'LinkedMDSExtendedPollActionResult':
    #
    #             decoded_message = self.decoder.readData(message)
    #             # This will send to splunk/file whatever
    #             # self.logger.info(decoded_message)
    #             #logging.info("Decoded message: {0}".format(decoded_message))
    #
    #             m = None # Secondary message decoding to "important stuff"
    #
    #             if decoded_message['PollMdibDataReplyExt']['Type']['OIDType'] == 'NOM_MOC_VMO_METRIC_SA_RT':
    #                 m = self.distiller.refine_wave_message(decoded_message)
    #
    #                 # To store and output message times (in order to log when to send Keep Alive Messages)
    #                 if decoded_message['ROapdus']['length'] > 100:
    #                     if 'RelativeTime' in decoded_message['PollMdibDataReplyExt'] and \
    #                                     decoded_message['PollMdibDataReplyExt']['sequence_no'] != 0:
    #                         self.messageTimes.append((decoded_message['PollMdibDataReplyExt'][
    #                                                       'RelativeTime'] - self.relativeInitialTime) / 8000)
    #                         # print(self.messageTimes[-1])
    #
    #                         # print('Received Monitor Data.')
    #             elif decoded_message['PollMdibDataReplyExt']['Type']['OIDType'] == 'NOM_MOC_VMO_METRIC_NU':
    #                 m = self.distiller.refine_numerics_message(decoded_message)
    #                 # print('Received Monitor Data.')
    #             elif decoded_message['PollMdibDataReplyExt']['Type']['OIDType'] == 'NOM_MOC_VMO_AL_MON':
    #                 m = self.distiller.refine_alarms_message(decoded_message)
    #                 # print('Received Alarm Data.')
    #
    #             if m:
    #                 mm = self.condense(m)
    #                 logging.info(mm)
    #
    #         else:
    #             logging.info('Received ' + message_type + '.')

    def close(self):
        """
        Sends Release Request and waits for confirmation, closes rs232 port
        """
        # Have to use `print` in here b/c logging may be gone if there is an error shutdown

        # If we have already closed or otherwise lost the port, pass and return
       
        if 0: # self.rs232 is None:
            logging.error('Trying to close a socket that no longer exists')
            raise IOError

        # Send Association Abort and Release Request
        self.rs232.send(self.AssociationAbort)
        logging.debug('Sent Association Abort...')
        self.rs232.send(self.ReleaseRequest)
        logging.debug('Sent Release Request...')

        not_refused = True

        # Loop to ensure breaking of connection
        count = 0
        while not_refused:
            message = self.rs232.receive()

            if not message:
                logging.debug('No release msg received!')
                break

            message_type = self.decoder.getMessageType(message)
            logging.debug('Received ' + message_type + '.')

            # If release response or association abort received, continue
            if message_type == 'ReleaseResponse' or message_type == 'AssociationAbort' or message_type == 'TimeoutError' or message_type == 'Unknown':
                logging.debug('Connection with monitor released.')
            elif count % 12 == 0:
                self.rs232.send(self.AssociationAbort)
                logging.debug('Re-sent Association Abort...')
                self.rs232.send(self.ReleaseRequest)
                logging.debug('Re-sent Release Request...')

            logging.debug('Trying to disconnect {0}'.format(count))
            count += 1

        self.rs232.close()
        self.rs232 = None

    def start_polling(self):
        """
        Sends Extended Poll Requests for Numeric, Alarm, and Wave Data
        """
        self.rs232.send(self.MDSExtendedPollActionNumeric)
        logging.debug('Sent MDS Extended Poll Action for Numerics...')
        self.rs232.send(self.MDSExtendedPollActionWave)
        logging.debug('Sent MDS Extended Poll Action for Waves...')
        self.rs232.send(self.MDSExtendedPollActionAlarm)
        logging.debug('Sent MDS Extended Poll Action for Alarms...')

    def single_poll(self):

        now = time.time()

        # Send keep alive if necessary
        if (now - self.last_keep_alive) > (self.KeepAliveTime - 5):
            self.submit_keep_alive()

        m = None

        message = self.rs232.receive()
        if not message:
            logging.warning('No message received')
            if (now - self.last_read_time) > self.timeout:
                logging.error('Data stream timed out')
                raise IOError
            return

        message_type = self.decoder.getMessageType(message)
        logging.debug(message_type)

        if message_type == 'AssociationAbort' or message_type == 'ReleaseResponse':
            logging.error('Received \'Data Collection Terminated\' message type.')
            # self.rs232.close()
            raise IOError

        # Apparently redundant
        # elif message_type == 'TimeoutError':
        #     if time.time() - self.last_read_time > self.timeout:
        #         self.close()
        #         raise IOError

        elif message_type == 'RemoteOperationError':
            logging.warning('Received (unhandled) \'RemoteOpsError\' message type')

        elif message_type == 'MDSSinglePollActionResult':
            logging.debug('Received (unhandled) \'SinglePollActionResult\' message type')

        elif message_type == 'MDSExtendedPollActionResult' or message_type == 'LinkedMDSExtendedPollActionResult':
            decoded_message = self.decoder.readData(message)
            m = self.distiller.refine(decoded_message)
            if not m:
                logging.warning('Failed to distill message: {0}'.format(decoded_message))
            else:
                self.last_read_time = time.time()

        else:
            logging.warning('Received {0}'.format(message_type))

        # Update current state
        if m:
            return self.condense(m)

    @staticmethod
    def condense(m):
        # Second pass distillation, from long intermediate format to condensed PERSEUS format

        # logging.debug(m)

        # This is 'NOM_ECG_ELEC_POTL_II' on my monitors, but let's map _any_ ECG wave label to ECG
        # especially b/c it seems to change to NOM_ECG_ELEC_POTL_V when leads are changed.
        ecg_label = None
        for key in m.keys():
            if 'ECG' in key:
                ecg_label = key
                break

        bp = {'systolic': m.get('non-invasive blood pressure_SYS'),
              'diastolic': m.get('non-invasive blood pressure_DIA'),
              'mean': m.get('non-invasive blood pressure_MEAN')}

        airway = {'etCO2': m.get('etCO2'),
                  'Respiration Rate': m.get('Airway Respiration Rate')}

        ret =  {'II': m.get(ecg_label),
                'Pleth': m.get('PLETH wave label'),
                'Heart Rate': m.get('Heart Rate'),
                'SpO2': m.get('Arterial Oxygen Saturation'),
                'Respiration Rate': m.get('Respiration Rate'),
                'Non-invasive Blood Pressure': bp,
                'Airway': airway,
                'alarms': m.get('alarms'),
                'timestamp': m.get('timestamp')}

        # TODO: Recursively go through ret and delete any None keys or {}...

        # logging.debug(ret)

        return ret

    # TelemetryStream parent class API
    def open(self, blocking=False): # 통신 시작지점 by JG

        opened = False

        while not opened:

            try:
                self.rs232 = RS232(self.port)        # This throws an error if it fails
                self.initiate_association(blocking)  # This tries to associate for 12 secs and then throws an error if it fails
                self.set_priority_lists()
                self.start_polling()
                self.last_read_time = time.time()
                opened = True

            except IOError:
                # Cool down period
                logging.error('Failed to open connection to {0}, waiting to try again'.format(self.port))
                time.sleep(1.0)
                self.close()
                pass

    def read(self, count=1, blocking=False):
        # Only read(1) is 'safe' and will block until it reconnects.

        if count < 0:
            # Read forever
            # self.extended_poll()
            logging.error('Extended poll is unimplemented in this version')
            raise NotImplementedError

        elif count == 0:
            return

        elif count == 1:
            try:
                data = self.single_poll()
                #self.logger.debug(data)

                # Update the sampled data buffer
                self.update_sampled_data(data) # Data Stacking by JG

                # Call any update functions in the order they were added
                if data:
                    for f in self.update_funcs:
                        new_data = f(sampled_data=self.sampled_data, **data)
                        data.update(new_data)

                # TODO: This should be sent to the data logger
                self.logger.info(data)
                return data
            except IOError:
                while 1:
                    logging.error('IOError reading stream, resetting connection')
                    try:
                        self.close()
                    except IOError:
                        logging.error('Ignoring IOError closing connection')

                    try:
                        self.open(blocking)
                        return self.read(1)
                    except IOError:
                        logging.error('IOError reestablishing connection, trying again')
                        time.sleep(1.0)
                        pass

        else:
            ret = []
            for i in range(0, count):
                data = self.read(1)
                if data:
                    ret.append(data)
            return ret

def update_plot(q_wave, q_ABPoutput, stop_event):
    def create_main_window():
        root = Toplevel()
        root.attributes('-fullscreen', True)  # 창을 전체 화면으로 설정
        root.attributes('-topmost', True)     # 창을 항상 위에 위치하도록 설정
        root.config(cursor="none")            # 마우스커서 숨기기
        root.overrideredirect(True)           # 타이틀 바 제거
        return root

    root = create_main_window()
    type_system = 1
   
    if type_system == 1: # for linux
        fontsize_default = 5
        fontsize_title = 4
        fontsize_numeric = 10
        fontsize_numeric_BP = 7.0
        margin_left_numtitle = -0.8
        margin_top_numtitle = 1.17
        margin_left_numeric = 0.2
        margin_top_numeric = 0.48
        margin_top_txtBP1 = 0.65
        margin_top_txtBP2 = 0.10
        margin_left_AIicon1 = -0.018
        margin_top_AIicon1 = 1.03
        margin_left_AIicon2 = -0.92
        margin_top_AIicon2 = 1.05
        icon_zoom = 0.038
        margin_left_FPS = 1.2
        margin_top_FPS = 4.25
        margin_wspace, margin_hspace = 0.95, 0.5
        margin_top, margin_bottom, margin_left, margin_right = 0.90, 0.05, 0.01, 0.95
       
    elif type_system == 2: # for surface
        fontsize_default = 6
        fontsize_title = 4
        fontsize_numeric = 12.5
        fontsize_numeric_BP = 9
        margin_left_numtitle = -0.2
        margin_top_numtitle = 1.09
        margin_left_numeric = 0.45
        margin_top_numeric = 0.58
        margin_top_txtBP1 = 0.75
        margin_top_txtBP2 = 0.35
        margin_left_AIicon1 = 0.0001
        margin_top_AIicon1 = 1.03
        margin_left_AIicon2 = -0.2
        margin_top_AIicon2 = 1.05
        icon_zoom = 0.04
        margin_left_FPS = 1.15
        margin_top_FPS = 3.7
        margin_wspace, margin_hspace = 0.35, 0.25
        margin_top, margin_bottom, margin_left, margin_right = 0.92, 0.02, 0.01, 0.97

    elif type_system == 3: # for window
        fontsize_default = 6
        fontsize_title = 4
        fontsize_numeric = 12.5
        fontsize_numeric_BP = 11
        margin_left_numtitle = -0.2
        margin_top_numtitle = 1.09
        margin_left_numeric = 0.45
        margin_top_numeric = 0.58
        margin_top_txtBP1 = 0.75
        margin_top_txtBP2 = 0.35
        margin_left_AIicon1 = 0.0001
        margin_top_AIicon1 = 1.03
        margin_left_AIicon2 = -0.2
        margin_top_AIicon2 = 1.06
        icon_zoom = 0.04
        margin_left_FPS = 1.15
        margin_top_FPS = 3.7
        margin_wspace, margin_hspace = 0.35, 0.25
        margin_top, margin_bottom, margin_left, margin_right = 0.92, 0.02, 0.01, 0.97
       
    image_path = 'AI_logo.png'  # 여기에 아이콘 이미지 경로를 입력하세요.
    icon_AI = mpimg.imread(image_path)
    imagebox = OffsetImage(icon_AI, zoom=icon_zoom)
   
    plt.style.use('dark_background')
   
    range_of = {'pleth': (0, 5000), 'ecg': (-1.5, 2), 'abp': (40, 140)}
    colors = ['lime', 'cyan', 'red', '#333333']

    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    fig = plt.Figure(figsize=(screen_width / 100, screen_height / 100))
    fig.subplots_adjust(wspace=margin_wspace, hspace=margin_hspace, top=margin_top, bottom=margin_bottom, left=margin_left, right=margin_right)
    gs = fig.add_gridspec(nrows=3, ncols=5)

    ## plot
    # ECG Wave
    ax_wECG = fig.add_subplot(gs[0, 0:4])
    ax_wECG.set_title("ECG", loc='left', fontweight='bold', color=colors[0], fontsize=fontsize_default * fontsize_title)
    ax_wECG.set_xticklabels([])
    ax_wECG.set_yticklabels([])
    ax_wECG.tick_params(axis='both', which='both', length=0)
    ax_wECG.grid(axis='y', color=colors[0], linestyle='--', linewidth=0.5)
    for spine in ax_wECG.spines.values():
        spine.set_visible(False)
    line_ecg, = ax_wECG.plot([], [], color=colors[0], linewidth=2)
    ax_wECG.set_ylim(range_of['ecg'])

    # PPG Wave
    ax_wPPG = fig.add_subplot(gs[1, 0:4])
    ax_wPPG.set_title("Pleth", loc='left', fontweight='bold', color=colors[1], fontsize=fontsize_default * fontsize_title)
    ax_wPPG.set_xticklabels([])
    ax_wPPG.set_yticklabels([])
    ax_wPPG.tick_params(axis='both', which='both', length=0)
    ax_wPPG.grid(axis='y', color=colors[1], linestyle='--', linewidth=0.5)
    for spine in ax_wPPG.spines.values():
        spine.set_visible(False)
    line_pleth, = ax_wPPG.plot([], [], color=colors[1], linewidth=2)
    ax_wPPG.set_ylim(range_of['pleth'])

    # ABP Wave
    ax_wABP = fig.add_subplot(gs[2, 0:4])
    ax_wABP.set_title("ABP", loc='left', fontweight='bold', color=colors[2], fontsize=fontsize_default * fontsize_title)
    art_icon1 = AnnotationBbox(imagebox, (margin_left_AIicon1, margin_top_AIicon1), xycoords='axes fraction', frameon=False, box_alignment=(-1.9, 0))
    ax_wABP.add_artist(art_icon1)
    ax_wABP.set_xticklabels([])
    ax_wABP.set_yticklabels([])
    ax_wABP.tick_params(axis='both', which='both', length=0)
    for spine in ax_wABP.spines.values():
        spine.set_visible(False)
    line_abp, = ax_wABP.plot([], [], color=colors[2], linewidth=2)

    ## text
    # HR Value Display
    ax_nECG = fig.add_subplot(gs[0, 4])
    ax_nECG.text(margin_left_numtitle, margin_top_numtitle, "HR", ha='left', va='center', color=colors[0], fontsize=fontsize_default * fontsize_title, fontweight='bold')
    txt_HR = ax_nECG.text(margin_left_numeric, margin_top_numeric, "-", ha='center', va='center', color=colors[0], fontsize=fontsize_default * fontsize_numeric)
    ax_nECG.axis('off')

    # SpO2 and BP Text Display
    ax_nPPG = fig.add_subplot(gs[1, 4])
    ax_nPPG.text(margin_left_numtitle, margin_top_numtitle, "SpO2", ha='left', va='center', color=colors[1], fontsize=fontsize_default * fontsize_title, fontweight='bold')
    txt_SPO2 = ax_nPPG.text(margin_left_numeric, margin_top_numeric, "-", ha='center', va='center', color=colors[1], fontsize=fontsize_default * fontsize_numeric)
    ax_nPPG.axis('off')

    ax_nBP = fig.add_subplot(gs[2, 4])
    ax_nBP.text(margin_left_numtitle, margin_top_numtitle, "ABP", ha='left', va='center', color=colors[2], fontsize=fontsize_default * fontsize_title, fontweight='bold')
    txt_SBPDBP = ax_nBP.text(margin_left_numeric + 0.05, margin_top_txtBP1, "- / -", ha='center', va='center', color=colors[2], fontsize=fontsize_default * fontsize_numeric_BP)
    txt_MAP = ax_nBP.text(margin_left_numeric + 0.05, margin_top_txtBP2, "(-)", ha='center', va='center', color=colors[2], fontsize=fontsize_default * fontsize_numeric_BP)
    ax_nBP.axis('off')
    art_icon2 = AnnotationBbox(imagebox, (margin_left_AIicon2, margin_top_AIicon2), xycoords='axes fraction', frameon=False, box_alignment=(-1.9, 0))
    ax_nBP.add_artist(art_icon2)

    # FPS
    txt_FPS = ax_nBP.text(margin_left_FPS, margin_top_FPS, "-/-", ha='right', va='center', color='black', fontsize=fontsize_default * fontsize_numeric_BP * 0.3)

    def ylim_auto(sig, gap_ratio):
        # None과 비숫자 값을 필터링
        valid_sig = [x for x in sig if x is not None and isinstance(x, (int, float))]
        
        if not valid_sig:
            return 0, 1  # 유효한 값이 없는 경우
        
        min_sig, max_sig = min(valid_sig), max(valid_sig)
        gap_sig = max_sig - min_sig
        min_ylim, max_ylim = min_sig - gap_ratio * gap_sig, max_sig + gap_ratio * gap_sig
        return min_ylim, max_ylim

    # ylim adjust
    def on_click_zoom(event, fig, axes):
        if event.inaxes in axes:
            ax = event.inaxes
            lines = ax.get_lines()
            if lines:
                ydata = np.concatenate([line.get_ydata() for line in lines])
                ylim_min, ylim_max = ylim_auto(ydata, 0.2)
                ax.set_ylim([ylim_min, ylim_max])

    # fps text toggle
    def toggle_fps(event, txt, fig):
        bbox = txt.get_window_extent(fig.canvas.get_renderer())
        if bbox.contains(event.x, event.y):
            current_color = txt.get_color()
            new_color = 'black' if current_color == 'white' else 'white'
            txt.set_color(new_color)

    # program exit event
    def on_close(event):
        print("Figure closed.")
        stop_event.set()

    # multi click event for exit
    global click_count, last_click_time
    click_count = 0
    last_click_time = time.time()
    def check_clicks_and_close(event, fig, axes):
        global click_count, last_click_time
        current_time = time.time()
        if current_time - last_click_time <= 1:
            click_count += 1
        else:
            click_count = 1  # 시간이 경과하면 클릭 횟수 초기화
            last_click_time = current_time

        if click_count >= 5:
            stop_event.set()
            fig.canvas.get_tk_widget().master.destroy()
               
    fig.canvas.mpl_connect('button_press_event', partial(on_click_zoom, fig=fig, axes=[ax_wECG, ax_wPPG, ax_wABP]))
    fig.canvas.mpl_connect('button_press_event', partial(toggle_fps, txt=txt_FPS, fig=fig))
    fig.canvas.mpl_connect('close_event', on_close)
    fig.canvas.mpl_connect('button_press_event', partial(check_clicks_and_close, fig=fig, axes=ax_nECG))

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.draw()
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    ## for realtime update
    buff_tdelta_ecg = deque(maxlen=3) # for timediff stacking
    buff_base_time_ecg = deque(maxlen=16) # for smoothing

    buff_tdelta_ppg = deque(maxlen=3) # for timediff stacking
    buff_base_time_ppg = deque(maxlen=16) # for smoothing
   
    w_avgs = 20
    buff_sBP = deque(maxlen=w_avgs)
    buff_dBP = deque(maxlen=w_avgs)
    buff_mBP = deque(maxlen=w_avgs)
    
    update_time_SPO2 = time.time()
    update_time_HR = time.time()
    textoff_time = 30
    
    base_time_ecg = time.time()-100
    base_time_ppg = time.time()-100
    t_pre = time.time()
    abplim_first = 1
    skip_frame = 3 # first 2 frame receive time unstable!!
    buff_frame = 0 # after 2 frame receive time average for init value!!

    while not stop_event.is_set():
        if not q_ABPoutput.empty(): 
            t_ecg, s_ecg, t_pleth, s_pleth, s_abp, predict_abp, HR, SPO2, t_receive, is_estiABP = q_ABPoutput.get(timeout=0)

            # time adjust for frame smoothing
            if skip_frame > 0:
#                print(skip_frame)
                skip_frame -= 1
            else:
                buff_tdelta_ecg.append(t_receive - t_ecg[-1])
                buff_tdelta_ppg.append(t_receive - t_pleth[-1])
                
                if skip_frame > -5:
                    ylim_min, ylim_max = ylim_auto(s_ecg, 0.2)
                    ax_wECG.set_ylim((ylim_min, ylim_max))
                    ylim_min, ylim_max = ylim_auto(s_pleth, 0.2)
                    ax_wPPG.set_ylim((ylim_min, ylim_max))
                    skip_frame -= 1

                if buff_frame > 0:
                    buff_frame -= 1
                else:
                    buff_base_time_ecg.append(max(buff_tdelta_ecg))
                    buff_base_time_ppg.append(max(buff_tdelta_ppg))
                    base_time_ecg = sum(buff_base_time_ecg) / len(buff_base_time_ecg)
                    base_time_ppg = sum(buff_base_time_ppg) / len(buff_base_time_ppg)
#                    print(base_time_ecg)

            # UI data update
            if not HR == 0:
                txt_HR.set_color(colors[0])
                txt_HR.set_text("{0:.0f}".format(HR))
                update_time_HR = time.time()
            else:
                if time.time() - update_time_HR >= textoff_time:
                    txt_HR.set_text("-")
                txt_HR.set_color(colors[3])
                
            if not SPO2 == 0:
                txt_SPO2.set_color(colors[1])
                txt_SPO2.set_text("{0:.0f}".format(SPO2))
                update_time_SPO2 = time.time()
            else:
                if time.time() - update_time_SPO2 >= textoff_time:
                    txt_SPO2.set_text("-")
                txt_SPO2.set_color(colors[3])
                
            line_ecg.set_data(t_ecg, s_ecg)
            line_pleth.set_data(t_pleth, s_pleth)

            # ABP data inable check
            if is_estiABP:
                line_abp.set_data(t_pleth, s_abp)
                
                if not SPO2 == 0:
                    buff_dBP.append(predict_abp[0])
                    buff_sBP.append(predict_abp[1])
                    buff_mBP.append(predict_abp[2])
                    avg_dBP = sum(buff_dBP) / len(buff_dBP)
                    avg_sBP = sum(buff_sBP) / len(buff_sBP)
                    avg_mBP = sum(buff_mBP) / len(buff_mBP)
                    
                    txt_SBPDBP.set_color(colors[2])
                    txt_MAP.set_color(colors[2])
                    txt_SBPDBP.set_text("{0:.0f}".format(avg_sBP) + "/" + "{0:.0f}".format(avg_dBP))
                    txt_MAP.set_text("({0:.0f})".format(avg_mBP))
                else:
                    if time.time() - update_time_SPO2 >= textoff_time:
                        txt_SBPDBP.set_text("- / -")
                        txt_MAP.set_text("(-)")
                    txt_SBPDBP.set_color(colors[3])
                    txt_MAP.set_color(colors[3])
                    
                # abp ylim init
                if abplim_first:
                    axis_min_abp_wave, axis_max_abp_wave = ylim_auto(s_abp, 0.2)
                    ax_wABP.set_ylim((axis_min_abp_wave, axis_max_abp_wave))
                    abplim_first = 0

        # xlim update
        t_update = time.time() - base_time_ecg - 1.7
        ax_wECG.set_xlim(t_update - 5, t_update)
        t_update = time.time() - base_time_ppg - 2.0
        ax_wPPG.set_xlim(t_update - 5, t_update)
        ax_wABP.set_xlim(t_update - 5, t_update)

        # for FPS update
        execution_time = time.time() - t_pre
        t_pre = time.time()
        if execution_time == 0:
            execution_time = 0.01
        txt_FPS.set_text("{0:.0f} fps".format(1 / execution_time))

        # frame update
        fig.canvas.draw()
        root.update_idletasks()
        root.update()

    root.mainloop()

def find_first_greater(arr, target):
    # bisect_right는 target보다 큰 첫 번째 요소의 인덱스를 반환합니다.
    index = bisect.bisect_right(arr, target)
    # index가 배열의 길이와 같으면, target보다 큰 요소가 없는 경우입니다.
    return index if index < len(arr) else -1

## ABP estimation part
def normalize_data(data, data_min, data_max, normalized_min, normalized_max):
    if data_min == data_max:
        print("Error: Data minimum and maximum values are equal.")
        return None

    if normalized_min >= normalized_max:
        print("Error: Normalization range is invalid.")
        return None

    # 데이터를 normalized_min 에서 normalized_max 로 정규화합니다.
    normalized_data = normalized_min + (data - data_min) * ((normalized_max - normalized_min) / (data_max - data_min))
    return normalized_data

def denormalize_data(normalized_data, data_min, data_max, normalized_min, normalized_max):
    if normalized_min >= normalized_max:
        print("Error: Normalization range is invalid.")
        return None

    if normalized_data is None:
        print("Error: Normalized data is None.")
        return None

    # 정규화된 데이터를 다시 원래 데이터로 복원합니다.
    data = ((normalized_data - normalized_min) * (data_max - data_min) / (normalized_max - normalized_min)) + data_min
    return data

# min/max ormalization
def minmax_normalize(signal):
    min_val = np.min(signal)
    max_val = np.max(signal)
    normalized_signal = (signal - min_val) / (max_val - min_val)
    return normalized_signal, min_val, max_val

# restoring function
def minmax_restore(normalized_signal, min_val, max_val):
    original_signal = normalized_signal * (max_val - min_val) + min_val
    return original_signal

def apply_band_pass_filter(data, lowcut, highcut, sampling_frequency, order=4, padlen=None):
    nyquist_frequency = 0.5 * sampling_frequency
    low = lowcut / nyquist_frequency
    high = highcut / nyquist_frequency

    b, a = butter(order, [low, high], btype='band', analog=False)

    if padlen is not None:
        f_data = filtfilt(b, a, data, padlen=padlen)
    else:
        f_data = filtfilt(b, a, data)

    return f_data

def apply_low_pass_filter(data, cutoff, sampling_frequency, order=4, padlen=None):
    nyquist_frequency = 0.5 * sampling_frequency
    normalized_cutoff = cutoff / nyquist_frequency

    b, a = butter(order, normalized_cutoff, btype='low', analog=False)

    if padlen is not None:
        f_data = filtfilt(b, a, data, padlen=padlen)
    else:
        f_data = filtfilt(b, a, data)

    return f_data

def check_none(arr):
    return np.any([element is None for element in arr])

def esti_ABP(q_ABPinput, q_ABPoutput, ABP_event):
    # global abp_model, wave_model
    # abp_model = tf.keras.models.load_model('ABP_model_tf')
    # wave_model = tf.keras.models.load_model('wave_model_tf')
    with open('ABP_model.tflite', 'rb') as f:
        abp_tflite_model = f.read()
    abpwave_interpreter = tf.lite.Interpreter(model_content=abp_tflite_model)
    abpwave_interpreter.allocate_tensors() 
    abpwave_input_details = abpwave_interpreter.get_input_details()
    abpwave_output_details = abpwave_interpreter.get_output_details()

    with open('wave_model.tflite', 'rb') as f:
        ppg_tflite_model = f.read()
    abpvalue_interpreter = tf.lite.Interpreter(model_content=ppg_tflite_model)
    abpvalue_interpreter.allocate_tensors() 
    abpvalue_input_details = abpvalue_interpreter.get_input_details()
    abpvalue_output_details = abpvalue_interpreter.get_output_details()

    def predict_wave_PPG(wave_PPG):
        input_data = np.expand_dims(np.array(wave_PPG, dtype=np.float32), axis=0)
        abpwave_interpreter.set_tensor(abpwave_input_details[0]['index'], input_data)
        abpwave_interpreter.invoke()
        predict_result = abpwave_interpreter.get_tensor(abpwave_output_details[0]['index'])
        return predict_result

    def predict_value_PPG(normalized_ppg):
        input_data = np.expand_dims(np.array(normalized_ppg, dtype=np.float32), axis=0)
        abpvalue_interpreter.set_tensor(abpvalue_input_details[0]['index'], input_data)
        abpvalue_interpreter.invoke()
        predict_result = abpvalue_interpreter.get_tensor(abpvalue_output_details[0]['index'])
        return predict_result
    
    # DBP/SBP/MAP Normaization
    abp_data_min = 20
    abp_data_max = 200
    abp_normalized_min = 0
    abp_normalized_max = 1

    # Cutoff frequency of lowpass filter
    cutoff_high = 12
    SAMPLING_RATE = 125

    while True:
        if not q_ABPinput.empty():
            wave_tECG, wave_ECG, wave_tPPG, wave_PPG, buff_HR, buff_SPO2, t_receive = q_ABPinput.get_nowait()
            if ~check_none(wave_PPG):
#                print(np.std(wave_PPG))
                predict_result = predict_wave_PPG(wave_PPG)
                predict_result = np.array(predict_result)
                predict_result = predict_result.reshape(-1)
                predict_abp = denormalize_data(predict_result, abp_data_min, abp_data_max, abp_normalized_min, abp_normalized_max)
       
                normalized_ppg, min_val, max_val = minmax_normalize(wave_PPG)
                results = predict_value_PPG(normalized_ppg)
                results = np.array(results)
                predictions = tf.squeeze(results, axis=-1)
                predict_wave = predictions [0]
                abp_wave = apply_low_pass_filter(predict_wave, cutoff_high, SAMPLING_RATE)

                # q_ABPoutput.put((predict_abp, abp_wave))
                q_ABPoutput.put((wave_tECG, wave_ECG, wave_tPPG, wave_PPG, abp_wave, predict_abp, buff_HR, buff_SPO2, t_receive, True))

            else:
                q_ABPoutput.put((wave_tECG, wave_ECG, wave_tPPG, wave_PPG, 0, 0, buff_HR, buff_SPO2, t_receive, False))

            ABP_event.clear()
               
if __name__ == '__main__':

    logging.basicConfig(level=logging.DEBUG)
    parser = argparse.ArgumentParser() # add parser by JG
    opts = parser.parse_known_args() # add parser by JG
    #opts = parse_args() # add parser by JG
   
    # opts.splunk = "perseus"
    # opts.gui = "SimpleStripchart"
    # ECG is 64 samples and Pleth is 32 samples every 0.25 secs
    # opts.values = ["Pleth", 32*4, 'ECG', 64*4]
    # Pleth _must_ be listed first if both Pleth and ECG are included

    def select_port(selected_port):
        global port_sel
        port_sel = selected_port
        print(f"선택한 COM 포트: {port_sel}")
        if root is not None:  # root가 None이 아닌 경우에만 quit 호출
            root.quit()  # Tkinter 루프 종료

    def show_no_ports_message():
        tk.Label(root, text="사용 중인 COM 포트가 없습니다.").pack()
        tk.Button(root, text="확인", command=lambda: os._exit(0)).pack()

    def show_single_port_message(port):
        global port_sel
        port_sel, desc, hwid = port
#        tk.Label(root, text=f"사용 중인 COM 포트: {port_sel}").pack()
#        tk.Button(root, text="확인", command=lambda: (root.quit(), root.destroy())).pack()

    port_sel = None  # 선택된 포트를 저장할 전역 변수
    root = None  # root 변수를 초기화
    
    def check_ports():
        ports = serial.tools.list_ports.comports()
        ports = [port for port in ports if not port.device.startswith('/dev/ttyAMA')]  # /dev/ttyAMA 포트를 제외

        if len(ports) == 1:
            # 단일 포트가 발견된 경우
            show_single_port_message(ports[0])  # 포트 정보 출력
            root.quit()  # 창 종료
        else:
            # 계속해서 포트를 확인 (1초 후 다시 체크)
            root.after(1000, check_ports)
            
    # 포트 체크
    ports = serial.tools.list_ports.comports()
    ports = [port for port in ports if not port.device.startswith('/dev/ttyAMA')]  # /dev/ttyAMA 포트를 제외

    # 1개의 포트가 있으면 창을 생성하지 않음
    if len(ports) == 1:
        show_single_port_message(ports[0])
    else:
        # 여러 포트가 발견되었을 때만 창을 생성
        root = tk.Tk()
        root.title("COM port connect")
        label = tk.Label(root, text="사용 중인 COM 포트가 없습니다")
        label.pack()

        # 포트를 주기적으로 체크
        check_ports()

        root.mainloop()
    # Tkinter 루프 종료 후 포트가 선택되었는지 확인
    if port_sel is not None:
        print(f"선택된 포트로 프로세스를 시작합니다: {port_sel}")
        # PhilipsTelemetryStream을 설정하는 코드
        tstream = PhilipsTelemetryStream(port=port_sel,                                          values=["Pleth", 32*4, 'II', 64*8], polling_interval=0.05)
    else:
        print("포트가 선택되지 않았습니다.")
        
    # ports = [port for port in ports if not port.device.startswith('/dev/ttyAMA')]  # /dev/ttyAMA 포트를 제외
            
    # Attach any post-processing functions
    tstream.add_update_func(qos)

    def watchdog():
        print("data communication error: program restart")
        q_wave.put('done')
        p_plot.terminate()
        p_ABP.terminate()
        tstream.close()
        sys.exit()
            
    if 0: # not opts.gui

        # Create a main loop that just echoes the results to the loggers
        tstream.run(blocking=True)

    else:
        # Pass the to a gui for use in it's own polling function and main loop
        # gui = TelemetryGUI(tstream, type=opts.gui, redraw_interval=0.05)
        # gui = TelemetryGUI(tstream) # by JG
        # gui.run(blocking=True)
        # global abp_model, wave_model
        # abp_model = tf.keras.models.load_model('ABP_model_tf')
        # wave_model = tf.keras.models.load_model('wave_model_tf')
   
        buff_tPPG = deque([0]*1024, maxlen=1024)
        buff_PPG = deque([None]*1024, maxlen=1024)
        buff_HR = 0
        buff_SPO2 = 0

        stop_event = mp.Event()
        q_wave = mp.Queue()
        q_ABPoutput = mp.Queue()
        p_plot = mp.Process(target=update_plot, args=(q_wave, q_ABPoutput, stop_event))
        p_plot.start()
       
        ABP_event = mp.Event()
        q_ABPinput = mp.Queue()
        # q_ABPoutput = mp.Queue()
        p_ABP = mp.Process(target=esti_ABP, args=(q_ABPinput,q_ABPoutput, ABP_event))
        p_ABP.start()
       
        redraw_interval = 0.1
        last_poll = time.time()
        t_lastTrans = time.time()
        # last_redraw = time.time()
        tstream.open()
       
        lastupdate_HR, lastupdate_SPO2 = 0, 0

        try:
            while not stop_event.is_set():
                now = time.time()
                if now > (last_poll + tstream.polling_interval):
                    timer = threading.Timer(5, watchdog)
                    timer.start()
                    data = tstream.read(1, blocking=False)
                    timer.cancel()
                    t_receive = time.time()
                    if data:
                        temp = list(tstream.sampled_data.keys())
                        HR = data.get('Heart Rate')
                        if HR:
                            buff_HR = HR
                            lastupdate_HR = time.time()
                        else:
                            if time.time() - lastupdate_HR > 2:
                                buff_HR = 0
                            
                        SPO2 = data.get('SpO2')
                        if SPO2:
                            buff_SPO2 = SPO2
                            lastupdate_SPO2 = time.time()
                        else:
                            if time.time() - lastupdate_SPO2 > 3:
                                buff_SPO2 = 0
                            
                        if ('II' in temp) and ('Pleth' in temp):
                            channel_ECG = tstream.sampled_data.get('II')
                            channel_PPG = tstream.sampled_data.get('Pleth')
                            if (not channel_ECG) or (not channel_PPG):
                                print("not channel")
                            else:
                                ECG = channel_ECG.get('samples')
                                PPG = channel_PPG.get('samples')
                                idx_update = find_first_greater(PPG.t, buff_tPPG[-1])
                                if idx_update >= 0:
                                    buff_tPPG.extend(PPG.t[idx_update:])
                                    buff_PPG.extend(PPG.y[idx_update:])
                                if not ABP_event.is_set() and (time.time() - t_lastTrans > 0.8):
                                    # q_wave.put((ECG.t, ECG.y, buff_tPPG, buff_PPG, buff_HR, buff_SPO2))
                                    ABP_event.set()
                                    q_ABPinput.put((ECG.t, ECG.y, buff_tPPG, buff_PPG, buff_HR, buff_SPO2,t_receive))
                                    t_lastTrans = time.time()
                        # q_ABPoutput.put((data.get('Heart Rate'), data.get('SpO2')))
                       
                        #self.display.update_data(data, tstream.sampled_data)
                        last_poll = now
                        #self.display.redraw()

                # if now > (last_redraw + redraw_interval):
                #     #self.display.redraw()
                #     last_redraw = now
                # time.sleep(redraw_interval/2)
            print("Process end")
            q_wave.put('done')
            p_plot.terminate()
            p_ABP.terminate()
            tstream.close()
            os._exit(0)

        except:
            print("Process exception occur!")
            q_wave.put('done')
            p_plot.terminate()
            p_ABP.terminate()
            tstream.close()
            os._exit(0)

# %%
