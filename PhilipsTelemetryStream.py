"""
@author: uagrawal last update: 2/29/16
@fixes: backport to 2.7 by derek

Requires pyserial (for RS232)
"""

# @leo/@uday/@derek
# TODO: How do we want to smooth the QoS values?

# Approaches to stability problems:
# TODO: log rotation, keep smaller files?
# TODO: Set priority to high/rt and see if it crashes?
# TODO: Turn the QoS function off and test?

from __future__ import unicode_literals

import argparse
import logging
import time
from IntellivueProtocol.IntellivueDecoder import IntellivueDecoder
from IntellivueProtocol.RS232 import RS232
from IntellivueProtocol.IntellivueDistiller import IntellivueDistiller
from TelemetryStream import TelemetryStream
from TelemetryStream import TelemetryGUI
from TelemetryStream import attach_loggers
from QualityOfSignal import QualityOfSignal as QoS
from collections import deque
import multiprocessing as mp
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

import easydict
args = easydict.EasyDict({
    "gui": "SimpleStripchart",
})

__description__ = "PERSEUS telemetry stream listener for Philips Invellivue devices with serial connections"
__version_info__ = ('0', '7', '3')
__version__ = '.'.join(__version_info__)

# updated 0.7.3 accounts for 8000Hz data collection throughout


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
                logging.warn('Trying to send an Association Request without a socket!')
                raise CriticalIOError
            try:
                self.rs232.send(self.AssociationRequest)
                self.logger.debug('Sent Association Request...')
            except:
                self.logger.warn("Unable to send Association Request")
                raise IOError

        def receive_association_response():
            association_message = self.rs232.receive()

            # Could handle no message in getMessageType (hrm)
            if not association_message:
                logging.warn('No association received')
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
                logging.warn('No priority list msg received!')
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
                logging.warn('Failed to confirm priority list setting.')

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
            logging.warn('No message received')
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
            logging.warn('Received (unhandled) \'RemoteOpsError\' message type')

        elif message_type == 'MDSSinglePollActionResult':
            logging.debug('Received (unhandled) \'SinglePollActionResult\' message type')

        elif message_type == 'MDSExtendedPollActionResult' or message_type == 'LinkedMDSExtendedPollActionResult':
            decoded_message = self.decoder.readData(message)
            m = self.distiller.refine(decoded_message)
            if not m:
                logging.warn('Failed to distill message: {0}'.format(decoded_message))
            else:
                self.last_read_time = time.time()

        else:
            logging.warn('Received {0}'.format(message_type))

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

        ret =  {'ECG': m.get(ecg_label),
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

def update_plot(q_wave,q_nums):
    plt.style.use('dark_background')
    
    range_of = {'pleth': (0,5000),'ecg': (-1.5, 2)}
    colors = ['lime', 'cyan', 'red']
    fontsize_default = 6
    margin_left_numtitle = -0.2
    margin_top_numtitle = 1.15
    margin_left_numeric = 0.2

    fig = plt.figure(figsize=(18,8))
    fig.subplots_adjust(wspace=0.25, hspace=0.5, top=0.90, bottom=0.05, left=0.01, right = 0.88)
    
    gs = plt.GridSpec(nrows=3, ncols=5)
    
    # ECG Wave
    ax_wECG = fig.add_subplot(gs[0,0:4])
    ax_wECG.set_title("ECG", loc='left', fontweight='bold', color=colors[0], fontsize= fontsize_default * 3)
    ax_wECG.set_xticklabels([])
    ax_wECG.set_yticklabels([])
    ax_wECG.tick_params(axis='both', which='both', length=0)
    ax_wECG.grid(axis='y', color=colors[0], linestyle='--', linewidth=0.5)
    for spine in ax_wECG.spines.values():
        spine.set_visible(False)
    line_ecg, = ax_wECG.plot([], [], color=colors[0], linewidth=2)
    
    # PPG Wave
    ax_wPPG = fig.add_subplot(gs[1,0:4])
    ax_wPPG.set_title("Pleth", loc='left', fontweight='bold', color=colors[1], fontsize= fontsize_default * 3)
    ax_wPPG.set_xticklabels([])
    ax_wPPG.set_yticklabels([])
    ax_wPPG.tick_params(axis='both', which='both', length=0)
    ax_wPPG.grid(axis='y', color=colors[1], linestyle='--', linewidth=0.5)
    for spine in ax_wPPG.spines.values():
        spine.set_visible(False)
    line_pleth, = ax_wPPG.plot([],[], color=colors[1], linewidth=2)
    
    # ABP Wave
    ax_wABP = fig.add_subplot(gs[2,0:4])
    ax_wABP.set_title("ABP", loc='left', fontweight='bold', color=colors[2], fontsize= fontsize_default * 3)
    ax_wABP.set_xticklabels([])
    ax_wABP.set_yticklabels([])
    ax_wABP.tick_params(axis='both', which='both', length=0)
    ax_wABP.grid(axis='y', color=colors[2], linestyle='--', linewidth=0.5)
    for spine in ax_wABP.spines.values():
        spine.set_visible(False)
    line_abp, = ax_wABP.plot([],[], color=colors[2], linewidth=2)
    
    ## text
    # HR Value Display
    ax_nECG = fig.add_subplot(gs[0, 4])
    ax_nECG.text(margin_left_numtitle, margin_top_numtitle, "HR", ha='left', va='center', color=colors[0], fontsize=fontsize_default*3, fontweight='bold')
    txt_HR = ax_nECG.text(margin_left_numeric, 0.5, "75", ha='left', va='center', color=colors[0], fontsize=fontsize_default*10)
    ax_nECG.axis('off')

    # SpO2 and BP Text Display
    ax_nPPG = fig.add_subplot(gs[1, 4])
    ax_nPPG.text(margin_left_numtitle, margin_top_numtitle, "SpO2", ha='left', va='center', color=colors[1], fontsize=fontsize_default*3, fontweight='bold')
    txt_SPO2 = ax_nPPG.text(margin_left_numeric, 0.5, "100", ha='left', va='center', color=colors[1], fontsize=fontsize_default*10)
    ax_nPPG.axis('off')

    ax_nBP = fig.add_subplot(gs[2, 4])
    ax_nBP.text(margin_left_numtitle, margin_top_numtitle, "BP", ha='left', va='center', color=colors[2], fontsize=fontsize_default*3, fontweight='bold')
    txt_SBPDBP = ax_nBP.text(margin_left_numeric, 0.75, "-/-", ha='left', va='center', color=colors[2], fontsize=fontsize_default*6)
    txt_MAP = ax_nBP.text(margin_left_numeric + 0.05, 0.25, "(-)", ha='left', va='center', color=colors[2], fontsize=fontsize_default*6)
    ax_nBP.axis('off')
    
    # text_nums = fig.text(0.79, 0.95,
    #             'HR: \nSpO2:',
    #             horizontalalignment='left',
    #             verticalalignment='center',
    #             fontsize=11)
        
    plt.draw()
    
    cnt = 0
    buff_ECG = deque([None]*1792, maxlen=1792)
    buff_tECG = deque([0]*1792, maxlen=1792)
    buff_PPG = deque([None]*896, maxlen=896)
    t_curr = 0
    
    while True:
        t_ecg, s_ecg, t_pleth, s_pleth, HR, SPO2 = q_wave.get() # waiting for queue data
        
        if HR and SPO2:
            txt_HR.set_text("{0:.0f}".format(HR))
            txt_SPO2.set_text("{0:.0f}".format(SPO2))
            txt_MAP.set_text(cnt)
            
            # text_nums.set_text("HR:  {0:.0f}\nSpO2: {1:.0f}".format(nums_HR, nums_SPO2))
            
        idx_curr = next((i for i, num in enumerate(t_ecg) if num > t_curr), 1791)
        rem_idx = 1792-idx_curr
        n_skip = 32
        cnt = 0
        start_time = time.time()
        while rem_idx > 0:
            buff_ECG.append(s_ecg[idx_curr])
            buff_tECG.append(t_ecg[idx_curr])
            idx_curr, rem_idx, cnt = idx_curr + 1, rem_idx - 1, cnt + 1
            if cnt > n_skip:
                line_ecg.set_data(list(buff_tECG), list(buff_ECG))
                ax_wECG.set_xlim(buff_tECG[0], buff_tECG[-1])
                plt.draw()
                plt.pause(0.001)
                cnt = cnt - n_skip
        end_time = time.time()
        execution_time = end_time - start_time
        print(f"Code execution time: {execution_time} seconds")
            # print("!!!!")
        # print(t_ecg) # 1792
        # print(t_pleth) # 896
        
        # line_ecg.set_data(t_ecg, s_ecg)
        # line_pleth.set_data(t_pleth, s_pleth)
        
        
        ax_wECG.set_ylim(range_of['ecg'])
        
        # ax_wPPG.set_xlim(t_pleth.min(), t_pleth.max())
        ax_wPPG.set_ylim(range_of['pleth'])
        
        # plt.draw()
        # plt.pause(0.001)
        
        
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

    tstream = PhilipsTelemetryStream(port='COM4',
                                     values=["Pleth", 32*4, 'ECG', 64*4],
                                     polling_interval=0.05)

    # Attach any post-processing functions
    tstream.add_update_func(qos)
    attach_loggers(tstream, opts)

    if 0: # not opts.gui

        # Create a main loop that just echoes the results to the loggers
        tstream.run(blocking=True)

    else:
        # Pass the to a gui for use in it's own polling function and main loop
        # gui = TelemetryGUI(tstream, type=opts.gui, redraw_interval=0.05)
        # gui = TelemetryGUI(tstream) # by JG
        # gui.run(blocking=True)
        q_wave = mp.Queue()
        q_nums = mp.Queue()
        p_plot = mp.Process(target=update_plot, args=(q_wave,q_nums))
        p_plot.start()
        
        redraw_interval = 0.1
        last_poll = time.time()
        # last_redraw = time.time()
        tstream.open()
        
        try:
            while 1:
                now = time.time()
                if now > (last_poll + tstream.polling_interval):
                    data = tstream.read(1, blocking=True)
                    if data:
                        temp = list(tstream.sampled_data.keys())
                        HR = data.get('Heart Rate')
                        SPO2 = data.get('SpO2')
                        
                        if ('ECG' in temp) and ('Pleth' in temp):
                            channel_ECG = tstream.sampled_data.get('ECG')
                            channel_PPG = tstream.sampled_data.get('Pleth')
                            if (not channel_ECG) or (not channel_PPG):
                                print("not channel")
                            else:
                                ECG = channel_ECG.get('samples')
                                PPG = channel_PPG.get('samples')

                                q_wave.put((ECG.t, ECG.y, PPG.t, PPG.y, HR, SPO2))
                        
                        # q_nums.put((data.get('Heart Rate'), data.get('SpO2')))
                        
                        #self.display.update_data(data, tstream.sampled_data)
                        last_poll = now
                        #self.display.redraw()

                # if now > (last_redraw + redraw_interval):
                #     #self.display.redraw()
                #     last_redraw = now
                # time.sleep(redraw_interval/2)
            
        except:
            q_wave.put('done')
            p_plot.join()
            tstream.close()

# %%
