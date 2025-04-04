from __future__ import unicode_literals

import argparse
import logging
import time
from IntellivueProtocol.IntellivueDecoder import IntellivueDecoder
from IntellivueProtocol.RS232 import RS232
from IntellivueProtocol.IntellivueDistiller import IntellivueDistiller
from TelemetryStream import TelemetryStream
from QualityOfSignal import QualityOfSignal as QoS
from collections import deque
import multiprocessing as mp
import numpy as np
import bisect
import tkinter as tk
from tkinter import Toplevel
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from scipy import signal
from scipy.signal import butter, filtfilt
from functools import partial
from itertools import cycle
from time import sleep
from gpiozero import PWMOutputDevice
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

logging.disable()

def qos(*args, **kwargs):
    """
    Wrapper for UCSF QoS code
    """
    my_qos = QoS()
    history = kwargs.get('sampled_data')
    if history:
        res = my_qos.isPPGGoodQuality(history.get('Pleth').get('samples').y, 125)
        return {'qos': res}
    else:
        return -1

class CriticalIOError(IOError):
    """Need to tear the socket down and reset."""
    pass


class PhilipsTelemetryStream(TelemetryStream):
    """
    Intellivue(Philips) 모니터와 RS232로 통신하는 클래스.
    TelemetryStream을 상속하여 필요함수 override.
    """

    def __init__(self, *args, **kwargs):
        super(PhilipsTelemetryStream, self).__init__(*args, **kwargs)

        self.logger.name = 'PhilipsTelemetry'

        serialPort = kwargs.get('port')
        selectedDataTypes = kwargs.get('values')[::2]

        self.port = serialPort
        self.rs232 = None

        # Initialize Intellivue Decoder and Distiller
        self.decoder = IntellivueDecoder()
        self.distiller = IntellivueDistiller()

        # Data collection params
        self.dataCollectionTime = 72 * 60 * 60  # seconds
        self.dataCollection = {'RelativeTime': self.dataCollectionTime * 8000}
        self.KeepAliveTime = 0
        self.messageTimes = []
        self.desiredWaveParams = {'TextIdLabel': selectedDataTypes}
        self.initialTime = 0
        self.relativeInitialTime = 0

        # Messages
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
        self.MDSExtendedPollActionNumeric = self.decoder.writeData(
            'MDSExtendedPollActionNUMERIC', self.dataCollection)
        self.MDSExtendedPollActionWave = self.decoder.writeData(
            'MDSExtendedPollActionWAVE', self.dataCollection)
        self.MDSExtendedPollActionAlarm = self.decoder.writeData(
            'MDSExtendedPollActionALARM', self.dataCollection)
        self.KeepAliveMessage = self.decoder.writeData('MDSSinglePollAction')

        self.data_flow = False
        self.last_read_time = time.time()
        self.timeout = 10
        self.last_keep_alive = time.time()

    def initiate_association(self, blocking=False):
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
            if not association_message:
                logging.warning('No association received')
                raise IOError

            message_type = self.decoder.getMessageType(association_message)
            self.logger.debug('Received ' + message_type + '.')

            if message_type == 'AssociationResponse':
                return association_message
            elif message_type == 'TimeoutError':
                raise IOError
            elif message_type in ['AssociationAbort', 'ReleaseRequest', 'Unknown']:
                raise CriticalIOError
            elif message_type in ['MDSExtendedPollActionResult', 'LinkedMDSExtendedPollActionResult']:
                raise CriticalIOError
            else:
                raise IOError

        def receive_event_creation(association_message):
            event_message = self.rs232.receive()
            message_type = self.decoder.getMessageType(event_message)
            logging.debug('Received ' + message_type + '.')

            if message_type == 'MDSCreateEvent':
                self.AssociationResponse = self.decoder.readData(association_message)
                logging.debug("Association response: {0}".format(self.AssociationResponse))

                self.KeepAliveTime = (
                    self.AssociationResponse['AssocRespUserData']['MDSEUserInfoStd']['supported_aprofiles']
                    ['AttributeList']['AVAType']['NOM_POLL_PROFILE_SUPPORT']['AttributeValue']
                    ['PollProfileSupport']['min_poll_period']['RelativeTime'] / 8000
                )
                self.MDSCreateEvent, self.MDSParameters = self.decoder.readData(event_message)
                self.initialTime = (
                    self.MDSCreateEvent['MDSCreateInfo']['MDSAttributeList']['AttributeList']['AVAType']
                    ['NOM_ATTR_TIME_ABS']['AttributeValue']['AbsoluteTime']
                )
                self.relativeInitialTime = (
                    self.MDSCreateEvent['MDSCreateInfo']['MDSAttributeList']['AttributeList']['AVAType']
                    ['NOM_ATTR_TIME_REL']['AttributeValue']['RelativeTime']
                )
                if 'saveInitialTime' in dir(self.distiller):
                    self.distiller.saveInitialTime(self.initialTime, self.relativeInitialTime)

                self.MDSCreateEventResult = self.decoder.writeData('MDSCreateEventResult', self.MDSParameters)
                self.rs232.send(self.MDSCreateEventResult)
                logging.debug('Sent MDS Create Event Result...')
                return
            else:
                self.logger.error('Bad handshake!')
                raise CriticalIOError

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

    def set_priority_lists(self):
        self.MDSSetPriorityListWave = self.decoder.writeData('MDSSetPriorityListWAVE', self.desiredWaveParams)
        self.rs232.send(self.MDSSetPriorityListWave)
        logging.debug('Sent MDS Set Priority List Wave...')

        no_confirmation = True
        while no_confirmation:
            message = self.rs232.receive()
            if not message:
                logging.warning('No priority list msg received!')
                break

            message_type = self.decoder.getMessageType(message)
            if message_type == 'MDSSetPriorityListResult':
                PriorityListResult = self.decoder.readData(message)
                if 'NOM_ATTR_POLL_RTSA_PRIO_LIST' in PriorityListResult['SetResult']['AttributeList']['AVAType']:
                    self.MDSSetPriorityListResultWave = PriorityListResult
                    logging.debug('Received MDS Set Priority List Result Wave.')
                no_confirmation = False
            elif message_type == 'MDSCreateEvent':
                no_confirmation = False
                logging.warning('Failed to confirm priority list setting.')

    def submit_keep_alive(self):
        self.rs232.send(self.KeepAliveMessage)
        self.last_keep_alive = time.time()
        logging.debug('Sent Keep Alive Message...')

    def close(self):
        if 0:  # self.rs232 is None:
            logging.error('Trying to close a socket that no longer exists')
            raise IOError

        self.rs232.send(self.AssociationAbort)
        logging.debug('Sent Association Abort...')
        self.rs232.send(self.ReleaseRequest)
        logging.debug('Sent Release Request...')

        not_refused = True
        count = 0
        while not_refused:
            message = self.rs232.receive()
            if not message:
                logging.debug('No release msg received!')
                break
            message_type = self.decoder.getMessageType(message)
            logging.debug('Received ' + message_type + '.')

            if message_type in ['ReleaseResponse', 'AssociationAbort', 'TimeoutError', 'Unknown']:
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
        self.rs232.send(self.MDSExtendedPollActionNumeric)
        logging.debug('Sent MDS Extended Poll Action for Numerics...')
        self.rs232.send(self.MDSExtendedPollActionWave)
        logging.debug('Sent MDS Extended Poll Action for Waves...')
        self.rs232.send(self.MDSExtendedPollActionAlarm)
        logging.debug('Sent MDS Extended Poll Action for Alarms...')

    def single_poll(self):
        now = time.time()
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

        if message_type in ['AssociationAbort', 'ReleaseResponse']:
            logging.error('Received \'Data Collection Terminated\' message type.')
            raise IOError
        elif message_type == 'RemoteOperationError':
            logging.warning('Received (unhandled) \'RemoteOpsError\' message type')
        elif message_type == 'MDSSinglePollActionResult':
            logging.debug('Received (unhandled) \'SinglePollActionResult\' message type')
        elif message_type in ['MDSExtendedPollActionResult', 'LinkedMDSExtendedPollActionResult']:
            decoded_message = self.decoder.readData(message)
            m = self.distiller.refine(decoded_message)
            if not m:
                logging.warning('Failed to distill message: {0}'.format(decoded_message))
            else:
                self.last_read_time = time.time()
        else:
            logging.warning('Received {0}'.format(message_type))

        if m:
            return self.condense(m)

    @staticmethod
    def condense(m):
        ecg_label = None
        for key in m.keys():
            if 'ECG' in key:
                ecg_label = key
                break

        bp = {
            'systolic': m.get('non-invasive blood pressure_SYS'),
            'diastolic': m.get('non-invasive blood pressure_DIA'),
            'mean': m.get('non-invasive blood pressure_MEAN')
        }

        airway = {
            'etCO2': m.get('etCO2'),
            'Respiration Rate': m.get('Airway Respiration Rate')
        }

        ret = {
            'II': m.get(ecg_label),
            'Pleth': m.get('PLETH wave label'),
            'Heart Rate': m.get('Heart Rate'),
            'SpO2': m.get('Arterial Oxygen Saturation'),
            'Respiration Rate': m.get('Respiration Rate'),
            'Non-invasive Blood Pressure': bp,
            'Airway': airway,
            'alarms': m.get('alarms'),
            'timestamp': m.get('timestamp')
        }
        return ret

    def open(self, blocking=False):
        opened = False
        while not opened:
            try:
                self.rs232 = RS232(self.port)
                self.initiate_association(blocking)
                self.set_priority_lists()
                self.start_polling()
                self.last_read_time = time.time()
                opened = True
            except IOError:
                logging.error('Failed to open connection to {0}, waiting to try again'.format(self.port))
                time.sleep(1.0)
                self.close()
                pass

    def read(self, count=1, blocking=False):
        if count == 1:
            try:
                data = self.single_poll()
                self.update_sampled_data(data)
                if data:
                    for f in self.update_funcs:
                        new_data = f(sampled_data=self.sampled_data, **data)
                        data.update(new_data)
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
                d = self.read(1, blocking=blocking)
                if d:
                    ret.append(d)
            return ret


##################################################
# 알람 (GPIO) 코드 - 메인 프로세스에서만 사용
##################################################
def play_melody(repeat=2, rest=0.4, volume=0.8):
    global alarm_stop

    FREQ = {
        'C': 261.63,  
        'F': 349.22,  
        'A': 440.00 
    }

    melody = [
        ('C', 0.2), ('A', 0.2), ('F', 0.2), ('REST', 0.2),
        ('A', 0.2), ('F', 0.2), ('REST', 0.6),
        ('C', 0.2), ('A', 0.2), ('F', 0.2), ('REST', 0.2),
        ('A', 0.2), ('F', 0.2), ('REST', 1.2)
    ]
    
    SLEEP_STEP = 0.01
    BUZZER_PIN = 18
    
    buzzer = PWMOutputDevice(BUZZER_PIN, active_high=True, initial_value=0)  # 객체 생성
    
    try:
        for _ in range(repeat):
            if alarm_stop:
                break

            for note, duration in melody:
                if alarm_stop:
                    break

                if note == 'REST':
                    end_time = time.time() + duration
                    while time.time() < end_time:
                        if alarm_stop:
                            break
                        time.sleep(SLEEP_STEP)
                    buzzer.value = 0
                elif note in FREQ:
                    buzzer.value = 0
                    time.sleep(0.01)
                    buzzer.frequency = FREQ[note]
                    buzzer.value = volume

                    end_time = time.time() + duration
                    while time.time() < end_time:
                        if alarm_stop:
                            break
                        time.sleep(SLEEP_STEP)

                    buzzer.value = 0

                    if alarm_stop:
                        break

                    end_time2 = time.time() + 0.05
                    while time.time() < end_time2:
                        if alarm_stop:
                            break
                        time.sleep(SLEEP_STEP)

            if not alarm_stop:
                end_time_rest = time.time() + rest
                while time.time() < end_time_rest:
                    if alarm_stop:
                        break
                    time.sleep(SLEEP_STEP)

            if alarm_stop:
                break
    finally:
        print("buzzer off 1")
        buzzer.off()  # PWM 출력 종료
        buzzer.close()  # 리소스 해제
        del buzzer  # 객체 삭제

def alarm_thread_func():
    global alarm_playing, alarm_stop
    print("[DEBUG] alarm_thread_func() start")
    try:
        # 알람 조건이 해제될 때까지 계속 무한반복
        while not alarm_stop:
            # 멜로디 1회( repeat=2 번 반복 ) 재생
            play_melody(repeat=2, rest=0.9, volume=0.4)
            # 한 번 다 돌았어도 alarm_stop이 여전히 False라면 다음 루프로
            # 중간 중간에도 play_melody 내부에서 alarm_stop 체크하므로 즉시 정지 가능
    except Exception as e:
        print("[ERROR] Alarm thread error:", e)
    finally:
        alarm_playing = False
        alarm_stop = False
        print("[DEBUG] alarm_thread_func() end")

def trigger_alarm():
    global alarm_playing, alarm_stop
    if alarm_playing:
        return
    alarm_playing = True
    alarm_stop = False
    t = threading.Thread(target=alarm_thread_func, daemon=True)
    t.start()

def stop_alarm():
    global alarm_stop
    alarm_stop = True

##################################################
# Plot 프로세스 함수 - GUI 표시용
##################################################
def update_plot(q_wave, q_ABPoutput, stop_event, q_alarm_flag):

    from tkinter import Toplevel, PhotoImage, Button
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.offsetbox import OffsetImage, AnnotationBbox
    from collections import deque
    import numpy as np
    import math
    from functools import partial
    
    def create_main_window():
        root = Toplevel()
        root.attributes('-fullscreen', True)
        root.attributes('-topmost', True)
        root.config(cursor="none")
        root.overrideredirect(True)
        return root

    root = create_main_window()
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()

    root.geometry(f"{screen_width+2}x{screen_height+2}+0+0")

    # -------------------
    # 파라미터 세팅
    # -------------------
    global ALARM_THRESHOLDS
    ALARM_THRESHOLDS = {
        'HR_LOW': 50,
        'HR_HIGH':120,
        'SPO2_LOW':90,
        'SBP_LOW':90,
        'SBP_HIGH':160,
        'DBP_LOW':45,
        'DBP_HIGH':90,
        'MBP_LOW':70,
        'MBP_HIGH':110
    }
    
    type_system = 1
    if type_system == 1:  # for linux
        global fontsize_default, fontsize_title, fontsize_numeric, fontsize_numeric_BP
        global margin_left_numtitle, margin_top_numtitle, margin_left_numeric, margin_top_numeric
        global margin_top_txtBP1, margin_top_txtBP2
        global margin_left_AIicon1, margin_top_AIicon1, margin_left_AIicon2, margin_top_AIicon2
        global icon_zoom, margin_left_FPS, margin_top_FPS
        global margin_wspace, margin_hspace
        global margin_top, margin_bottom, margin_left, margin_right

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
        margin_top, margin_bottom, margin_left, margin_right = 0.90, 0.05, 0.12, 0.95

        margin_icon_left = 10
        margin_icon_top = 60
        margin_icon_gap = 100
        
    image_path = 'AI_logo.png'
    icon_AI = mpimg.imread(image_path)
    imagebox = OffsetImage(icon_AI, zoom=icon_zoom)

    plt.style.use('dark_background')
    range_of = {'pleth': (0, 5000), 'ecg': (-1.5, 2), 'abp': (40, 140)}
    colors = ['lime', 'cyan', 'red', '#333333']

    fig = plt.Figure(figsize=(screen_width / 100, screen_height / 100))
    fig.subplots_adjust(wspace=margin_wspace, hspace=margin_hspace,
                        top=margin_top, bottom=margin_bottom,
                        left=margin_left, right=margin_right)
    gs = fig.add_gridspec(nrows=3, ncols=5)

    # -----------------------------
    #  ECG Plot
    # -----------------------------
    ax_wECG = fig.add_subplot(gs[0, 0:4])
    ax_wECG.set_title("ECG", loc='left', fontweight='bold', color=colors[0],
                      fontsize=fontsize_default * fontsize_title)
    ax_wECG.set_xticklabels([])
    ax_wECG.set_yticklabels([])
    ax_wECG.tick_params(axis='both', which='both', length=0)
    ax_wECG.grid(axis='y', color=colors[0], linestyle='--', linewidth=0.5)
    for spine in ax_wECG.spines.values():
        spine.set_visible(False)
    line_ecg, = ax_wECG.plot([], [], color=colors[0], linewidth=2)
    ax_wECG.set_ylim(range_of['ecg'])

    # -----------------------------
    #  PPG Plot
    # -----------------------------
    ax_wPPG = fig.add_subplot(gs[1, 0:4])
    ax_wPPG.set_title("Pleth", loc='left', fontweight='bold', color=colors[1],
                      fontsize=fontsize_default * fontsize_title)
    ax_wPPG.set_xticklabels([])
    ax_wPPG.set_yticklabels([])
    ax_wPPG.tick_params(axis='both', which='both', length=0)
    ax_wPPG.grid(axis='y', color=colors[1], linestyle='--', linewidth=0.5)
    for spine in ax_wPPG.spines.values():
        spine.set_visible(False)
    line_pleth, = ax_wPPG.plot([], [], color=colors[1], linewidth=2)
    ax_wPPG.set_ylim(range_of['pleth'])

    # -----------------------------
    #  ABP Plot
    # -----------------------------
    ax_wABP = fig.add_subplot(gs[2, 0:4])
    ax_wABP.set_title("ABP", loc='left', fontweight='bold', color=colors[2],
                      fontsize=fontsize_default * fontsize_title)
    art_icon1 = AnnotationBbox(imagebox, (margin_left_AIicon1, margin_top_AIicon1),
                               xycoords='axes fraction', frameon=False, box_alignment=(-1.9, 0))
    ax_wABP.add_artist(art_icon1)
    ax_wABP.set_xticklabels([])
    ax_wABP.set_yticklabels([])
    ax_wABP.tick_params(axis='both', which='both', length=0)
    for spine in ax_wABP.spines.values():
        spine.set_visible(False)
    line_abp, = ax_wABP.plot([], [], color=colors[2], linewidth=2)

    # -----------------------------
    #  Numeric Text (HR, SpO2, ABP)
    # -----------------------------
    ax_nECG = fig.add_subplot(gs[0, 4])
    ax_nECG.text(margin_left_numtitle, margin_top_numtitle, "HR",
                 ha='left', va='center', color=colors[0],
                 fontsize=fontsize_default * fontsize_title, fontweight='bold')
    txt_HR = ax_nECG.text(margin_left_numeric, margin_top_numeric, "-",
                          ha='center', va='center', color=colors[0],
                          fontsize=fontsize_default * fontsize_numeric)
    ax_nECG.axis('off')

    ax_nPPG = fig.add_subplot(gs[1, 4])
    ax_nPPG.text(margin_left_numtitle, margin_top_numtitle, "SpO2",
                 ha='left', va='center', color=colors[1],
                 fontsize=fontsize_default * fontsize_title, fontweight='bold')
    txt_SPO2 = ax_nPPG.text(margin_left_numeric, margin_top_numeric, "-",
                            ha='center', va='center', color=colors[1],
                            fontsize=fontsize_default * fontsize_numeric)
    ax_nPPG.axis('off')

    ax_nBP = fig.add_subplot(gs[2, 4])
    ax_nBP.text(margin_left_numtitle, margin_top_numtitle, "ABP",
                ha='left', va='center', color=colors[2],
                fontsize=fontsize_default * fontsize_title, fontweight='bold')
    txt_SBPDBP = ax_nBP.text(margin_left_numeric + 0.05, margin_top_txtBP1, "- / -",
                             ha='center', va='center', color=colors[2],
                             fontsize=fontsize_default * fontsize_numeric_BP)
    txt_MAP = ax_nBP.text(margin_left_numeric + 0.05, margin_top_txtBP2, "(-)",
                          ha='center', va='center', color=colors[2],
                          fontsize=fontsize_default * fontsize_numeric_BP)
    ax_nBP.axis('off')
    art_icon2 = AnnotationBbox(imagebox, (margin_left_AIicon2, margin_top_AIicon2),
                               xycoords='axes fraction', frameon=False, box_alignment=(-1.9, 0))
    ax_nBP.add_artist(art_icon2)

    txt_FPS = ax_nBP.text(margin_left_FPS, margin_top_FPS, "-/-",
                          ha='right', va='center', color='black',
                          fontsize=fontsize_default * fontsize_numeric_BP * 0.3)

    # -------------------------------------
    #  유틸 함수들 (zoom, fps toggle 등)
    # -------------------------------------
    def ylim_auto(sig, gap_ratio):
        valid_sig = [x for x in sig if x is not None and isinstance(x, (int, float))]
        if not valid_sig:
            return 0, 1
        
        # 피스메이커 신호 감지 (1000 이상의 값이 존재하며, 시계열의 중앙값이 100 이하인 경우)
        
        if max(valid_sig) >= 100 and min(valid_sig) <= 10:
            # 피스메이커 신호 제거: 100 이상인 값과 그 인접 값도 제외
            filtered_sig = []
            skip_next = False
            for i in range(len(valid_sig)):
                if skip_next:
                    skip_next = False
                    continue
                if valid_sig[i] >= 100:
                    if filtered_sig:
                        filtered_sig.pop # remove before
                    skip_next = True  # remove next
                    continue
                filtered_sig.append(valid_sig[i])
            
            if not filtered_sig:
                return 0, 1
            
            mn, mx = min(filtered_sig), max(filtered_sig)
            print('max value')
            print(mx)
            mx = min(mx * 3, 1000)  # R 피크를 고려하여 최대값의 3배 제한
        else:
            mn, mx = min(valid_sig), max(valid_sig)
        
        gap = mx - mn
        if gap <= 0 or math.isnan(mn) or math.isinf(mn) or math.isnan(mx) or math.isinf(mx):
            return 0, 1
        
        return (mn - gap_ratio * gap), (mx + gap_ratio * gap)
        
    import numpy as np

    def on_click_zoom(event, fig, axes):
        if not flag_lock:
            if event.inaxes in axes:
                ax = event.inaxes
                lines = ax.get_lines()
                if lines:
                    ydata = np.concatenate([line.get_ydata() for line in lines])
                    ymin, ymax = ylim_auto(ydata, 0.2)
                    ax.set_ylim([ymin, ymax])

    def toggle_fps(event, txt, fig):
        if not flag_lock:
            bbox = txt.get_window_extent(fig.canvas.get_renderer())
            import time
            if bbox.contains(event.x, event.y):
                current_color = txt.get_color()
                new_color = 'black' if current_color == 'white' else 'white'
                txt.set_color(new_color)

    def on_close(event):
        print("Figure closed.")
        stop_event.set()

    global click_count, last_click_time
    click_count = 0
    last_click_time = time.time()  # ← time.time() 호출

    def check_clicks_and_close(event, fig, axes):
        if not flag_lock:
            global click_count, last_click_time
            current_time = time.time()
            if current_time - last_click_time <= 1:
                click_count += 1
            else:
                click_count = 1
                last_click_time = current_time
            if click_count >= 5:
                stop_event.set()
                fig.canvas.get_tk_widget().master.destroy()

    # -----------------------------------
    #  이벤트 연결
    # -----------------------------------
    fig.canvas.mpl_connect(
        'button_press_event',
        partial(on_click_zoom, fig=fig, axes=[ax_wECG, ax_wPPG, ax_wABP])
    )
    fig.canvas.mpl_connect(
        'button_press_event',
        partial(toggle_fps, txt=txt_FPS, fig=fig)
    )
    fig.canvas.mpl_connect('close_event', on_close)
    fig.canvas.mpl_connect(
        'button_press_event',
        partial(check_clicks_and_close, fig=fig, axes=ax_nECG)
    )

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.draw()
    canvas.get_tk_widget().pack(side="top", fill="both", expand=1)

    # ------------------------------------------------
    #  알람 ON/OFF 버튼 추가
    # ------------------------------------------------
    alarm_enabled = True
    flag_lock = False
    flag_autoscale = False
    icon_scale = 3
    
    alarm_on_img = PhotoImage(file="icons/icon_Alarm_on.png").subsample(icon_scale, icon_scale)
    alarm_off_img = PhotoImage(file="icons/icon_Alarm_off.png").subsample(icon_scale, icon_scale)
    alarm_set_img = PhotoImage(file="icons/icon_Alarm_set.png").subsample(icon_scale, icon_scale)
    auto_scale_img = PhotoImage(file="icons/icon_Autoscale.png").subsample(icon_scale, icon_scale)
    lock_on_img = PhotoImage(file="icons/icon_Lock_on.png").subsample(icon_scale, icon_scale)
    lock_off_img = PhotoImage(file="icons/icon_Lock_off.png").subsample(icon_scale, icon_scale)
    
    def toggle_alarm_button():
        if not flag_lock:
            nonlocal alarm_enabled
            alarm_enabled = not alarm_enabled
            if alarm_enabled:
                button_alarm.config(image=alarm_on_img)
            else:
                button_alarm.config(image=alarm_off_img)
          
    # ------------------------------------------------
    #  Alarm Set 버튼 눌렀을 때 동작할 함수
    # ------------------------------------------------
    def alarmset_button():
        if not flag_lock:
            """
            메인화면(root) 내부에 Overlay Frame을 덮어씌우고,
            'Alarm Threshold' 팝업 인터페이스를 표시.
            Reset 버튼을 누르면 '윈도우 진입 시점'의 값으로 되돌린다.
            """
            global screen_width, screen_height
            global ALARM_THRESHOLDS

            # ---------------------------
            # 1) Overlay 생성
            # ---------------------------
            overlay = tk.Frame(root, bg="black")
            overlay.place(x=0, y=0, relwidth=1, relheight=1)
            overlay.lift()
            overlay.tkraise()
            root.update_idletasks()
            root.update()

            # ---------------------------
            # 2) Popup (gray) 프레임
            # ---------------------------
            popup_w = 600
            popup_h = 500
            popup = tk.Frame(overlay, bg="gray")
            popup.pack_propagate(False)
            popup.place(
                width=popup_w,
                height=popup_h,
                relx=0.5,
                rely=0.5,
                anchor="center"
            )

            label_title = tk.Label(
                popup,
                text="Set Alarm Thresholds",
                font=("Helvetica", 14, "bold"),
                fg="yellow",
                bg="gray"
            )
            label_title.pack(pady=10)

            # ---------------------------
            # [A] 로컬 스냅샷: 세팅 윈도우를 열었을 때 시점의 ALARM_THRESHOLDS
            # ---------------------------
            local_snapshot = ALARM_THRESHOLDS.copy()

            # ---------------------------
            # 3) 파라미터 목록
            #    (주의) 아래는 (title, high_key, low_key) 순서임
            # ---------------------------
            param_keys = [
                ("Heart Rate", "HR_HIGH", "HR_LOW"),
                ("SpO2",       "SPO2_LOW", None),
                ("Systolic",   "SBP_HIGH", "SBP_LOW"),
                ("Diastolic",  "DBP_HIGH", "DBP_LOW"),
                ("Mean BP",    "MBP_HIGH", "MBP_LOW"),
            ]

            param_frame = tk.Frame(popup, bg="gray")
            param_frame.pack(fill="both", expand=True, padx=10, pady=10)

            # 파라미터 라벨을 저장할 dict (key -> label 위젯)
            label_map = {}

            # + / - 버튼 클릭 시 ALARM_THRESHOLDS 갱신 + 라벨 업데이트
            def increment_value(key, delta, label_widget):
                ALARM_THRESHOLDS[key] += delta
                label_widget.config(text=str(ALARM_THRESHOLDS[key]))

            # ---------------------------
            # 4) 2열(Grid) 배치
            #    여기서는 첫 번째 인자: title, 두 번째: high_key, 세 번째: low_key
            #    UI에서는 "HIGH 먼저, LOW 나중" 순서로 표시
            # ---------------------------
            for i, (title, high_key, low_key) in enumerate(param_keys):
                row_idx = i // 2
                col_idx = i % 2

                frame_param = tk.Frame(param_frame, bg="gray", bd=1, relief="ridge")
                frame_param.grid(row=row_idx, column=col_idx, padx=8, pady=6, sticky="nsew")

                lbl_title = tk.Label(frame_param, text=title, fg="white", bg="gray",
                                     font=("Helvetica", 12, "bold"))
                lbl_title.pack(anchor="w", pady=2)

                # ---- HIGH threshold ----
                if high_key:
                    row_high = tk.Frame(frame_param, bg="gray")
                    row_high.pack(anchor="w", padx=8, pady=2)

                    # 현재(세팅 윈도우 열었을 때) 값
                    initial_val_high = ALARM_THRESHOLDS.get(high_key, 0)
                    lbl_highval = tk.Label(row_high, text=str(initial_val_high),
                                           fg="cyan", bg="gray", font=("Arial", 16, "bold"))


                    btn_high_minus = tk.Button(
                        row_high, text="-", width=3,
                        command=lambda k=high_key, l=lbl_highval: increment_value(k, -1, l)
                    )
                    btn_high_plus = tk.Button(
                        row_high, text="+", width=3,
                        command=lambda k=high_key, l=lbl_highval: increment_value(k, 1, l)
                    )

                    btn_high_minus.pack(side="left", padx=5)
                    btn_high_plus.pack(side="left", padx=5)

                    lbl_highname = tk.Label(row_high, text=f"{high_key}:", fg="white", bg="gray")
                    lbl_highname.pack(side="left", padx=5)
                    lbl_highval.pack(side="left", padx=5)
                    # label_map 저장
                    label_map[high_key] = lbl_highval

                # ---- LOW threshold ----
                if low_key:
                    row_low = tk.Frame(frame_param, bg="gray")
                    row_low.pack(anchor="w", padx=8, pady=2)

                    initial_val_low = ALARM_THRESHOLDS.get(low_key, 0)
                    lbl_lowval = tk.Label(row_low, text=str(initial_val_low),
                                          fg="cyan", bg="gray", font=("Arial", 16, "bold"))


                    btn_low_minus = tk.Button(
                        row_low, text="-", width=3,
                        command=lambda k=low_key, l=lbl_lowval: increment_value(k, -1, l)
                    )
                    btn_low_plus = tk.Button(
                        row_low, text="+", width=3,
                        command=lambda k=low_key, l=lbl_lowval: increment_value(k, 1, l)
                    )

                    btn_low_minus.pack(side="left", padx=5)
                    btn_low_plus.pack(side="left", padx=5)

                    lbl_lowname = tk.Label(row_low, text=f"{low_key}:", fg="white", bg="gray")
                    lbl_lowname.pack(side="left", padx=5)
                    lbl_lowval.pack(side="left", padx=5)
                    label_map[low_key] = lbl_lowval

            # ---------------------------
            # 5) Reset / Confirm
            # ---------------------------
            def confirm_and_close():
                # 지금까지 변경된 ALARM_THRESHOLDS가 그대로 유지
                overlay.destroy()

            def cancle_and_close():
                """
                세팅윈도우를 열었을 때(local_snapshot) 값으로 되돌린다
                + 라벨들도 다시 업데이트
                """
                ALARM_THRESHOLDS.clear()
                ALARM_THRESHOLDS.update(local_snapshot)

                # 저장된 label_map을 통해, 라벨 갱신
                for key, lbl in label_map.items():
                    lbl.config(text=str(ALARM_THRESHOLDS[key]))

            frame_btns = tk.Frame(popup, bg="gray")
            frame_btns.pack(pady=1)
            
            btn_confirm = tk.Button(
                frame_btns,
                text="Confirm",
                bg="green",
                fg="white",
                font=("Helvetica", 15, "bold"),
                command=confirm_and_close
            )
            btn_confirm.pack(side="left", padx=10)
            
            btn_cancel = tk.Button(
                frame_btns,
                text="Cancel",
                bg="red",
                fg="white",
                font=("Helvetica", 15, "bold"),
                command=cancle_and_close
            )
            btn_cancel.pack(side="left", padx=10)
            # ---------------------------
            # 6) 오버레이 다시 끌어올림
            # ---------------------------
            overlay.lift()
            overlay.tkraise()
            root.update_idletasks()
            root.update()

    def toggle_lock_button():
        nonlocal flag_lock
        flag_lock = not flag_lock
        if flag_lock:
            button_lock.config(image=lock_on_img)
        else:
            button_lock.config(image=lock_off_img)

    def autoscale_button():
        if not flag_lock:
            nonlocal flag_autoscale
            flag_autoscale = True
    
    # Alarm Button
    button_alarm = tk.Button(
        root,
        image=alarm_on_img,
        bd=0,
        highlightthickness=0,
        relief="flat",
        overrelief="flat",
        background="black",
        activebackground="black",
        takefocus=0,
        command=toggle_alarm_button
    )
    button_alarm.place(
        x=margin_icon_left,
        y=margin_icon_top,
        anchor="nw"
    )
    
    # Alarmset Button
    button_Alarmset = tk.Button(
        root,
        image=alarm_set_img,
        bd=0,
        highlightthickness=0,
        relief="flat",
        overrelief="flat",
        background="black",
        activebackground="black",
        takefocus=0,
        command=alarmset_button
    )
    button_Alarmset.place(
        x=margin_icon_left,
        y=margin_icon_top + margin_icon_gap,
        anchor="nw"
    )
    
    # Autoscale Button
    button_autoscale = tk.Button(
        root,
        image=auto_scale_img,
        bd=0,
        highlightthickness=0,
        relief="flat",
        overrelief="flat",
        background="black",
        activebackground="black",
        takefocus=0,
        command=autoscale_button
    )
    button_autoscale.place(
        x=margin_icon_left,
        y=margin_icon_top + 2*margin_icon_gap,
        anchor="nw"
    )
    
    # Lock Button
    button_lock = tk.Button(
        root,
        image=lock_off_img,
        bd=0,
        highlightthickness=0,
        relief="flat",
        overrelief="flat",
        background="black",
        activebackground="black",
        takefocus=0,
        command=toggle_lock_button
    )
    # 우측 상단
    button_lock.place(
        x=margin_icon_left,
        y=margin_icon_top + 3*margin_icon_gap,
        anchor="nw"
    )
    
    # ===============================================
    #   알람 깜빡임 관련 변수 (추가)
    # ===============================================
    # 항목별 알람 활성 상태
    hr_alarm_active   = False
    spo2_alarm_active = False
    sdbp_alarm_active   = False
    mbp_alarm_active   = False
    spo2_na = False
    hr_na = False
            
    # 항목별 blink 토글상태
    blink_HR_state   = False
    blink_SPO2_state = False
    blink_sdBP_state   = False
    blink_mBP_state   = False
    
    # 색상 정의
    color_HR_normal     = colors[0]  # lime
    color_HR_alarm      = 'yellow'
    color_HR_na         = 'gray'
    color_SPO2_normal   = colors[1]  # cyan
    color_SPO2_alarm    = 'yellow'
    color_SPO2_na       = 'gray'
    color_BP_normal     = colors[2]  # red
    color_BP_alarm      = 'yellow'
    color_BP_na         = 'gray'
    
    # 깜빡임 주기
    blink_interval = 0.5
    last_blink_time = time.time()

    # ===============================================
    #  주루프에서 사용하는 버퍼들
    # ===============================================
    
    buff_tdelta_ecg = deque(maxlen=3)
    buff_base_time_ecg = deque(maxlen=16)
    buff_tdelta_ppg = deque(maxlen=3)
    buff_base_time_ppg = deque(maxlen=16)

    w_avgs = 20
    buff_sBP = deque(maxlen=w_avgs)
    buff_dBP = deque(maxlen=w_avgs)
    buff_mBP = deque(maxlen=w_avgs)

    update_time_SPO2 = time.time()
    update_time_HR = time.time()
    textoff_time = 30

    base_time_ecg = time.time() - 100
    base_time_ppg = time.time() - 100
    t_pre = time.time()
    abplim_first = 1
    skip_frame = 3
    buff_frame = 0

    while not stop_event.is_set():
        if not q_ABPoutput.empty():
            (t_ecg, s_ecg, t_pleth, s_pleth,
             s_abp, predict_abp,
             HR, SPO2, t_receive, is_estiABP) = q_ABPoutput.get(timeout=0)

            if skip_frame > 0:
                skip_frame -= 1
            else:
                buff_tdelta_ecg.append(t_receive - t_ecg[-1])
                buff_tdelta_ppg.append(t_receive - t_pleth[-1])
                if skip_frame > -5:
                    ymin, ymax = ylim_auto(s_ecg, 0.2)
                    ax_wECG.set_ylim((ymin, ymax))
                    ymin, ymax = ylim_auto(s_pleth, 0.2)
                    ax_wPPG.set_ylim((ymin, ymax))
                    skip_frame -= 1
                if buff_frame > 0:
                    buff_frame -= 1
                else:
                    buff_base_time_ecg.append(max(buff_tdelta_ecg))
                    buff_base_time_ppg.append(max(buff_tdelta_ppg))
                    base_time_ecg = sum(buff_base_time_ecg) / len(buff_base_time_ecg)
                    base_time_ppg = sum(buff_base_time_ppg) / len(buff_base_time_ppg)

            # HR / SPO2 텍스트
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

            # ECG, PPG 파형 업데이트
            line_ecg.set_data(t_ecg, s_ecg)
            line_pleth.set_data(t_pleth, s_pleth)

            # ABP 파형
            if is_estiABP:
                line_abp.set_data(t_pleth, s_abp)
                if not SPO2 == 0:
                    buff_dBP.append(predict_abp[0])
                    buff_sBP.append(predict_abp[1])
                    buff_mBP.append(predict_abp[2])
                    avg_dBP = sum(buff_dBP)/len(buff_dBP)
                    avg_sBP = sum(buff_sBP)/len(buff_sBP)
                    avg_mBP = sum(buff_mBP)/len(buff_mBP)
                    txt_SBPDBP.set_color(colors[2])
                    txt_MAP.set_color(colors[2])
                    txt_SBPDBP.set_text("{0:.0f}/{1:.0f}".format(avg_sBP, avg_dBP))
                    txt_MAP.set_text("({0:.0f})".format(avg_mBP))
                else:
                    if time.time() - update_time_SPO2 >= textoff_time:
                        txt_SBPDBP.set_text("- / -")
                        txt_MAP.set_text("(-)")
                    txt_SBPDBP.set_color(colors[3])
                    txt_MAP.set_color(colors[3])
                    
                if abplim_first:
                    ymin, ymax = ylim_auto(s_abp, 0.2)
                    ax_wABP.set_ylim((ymin, ymax))
                    abplim_first = 0

            if flag_autoscale:
                ymin, ymax = ylim_auto(s_ecg, 0.2)
                ax_wECG.set_ylim((ymin, ymax))
                ymin, ymax = ylim_auto(s_pleth, 0.2)
                ax_wPPG.set_ylim((ymin, ymax))
                if is_estiABP:
                    ymin, ymax = ylim_auto(s_abp, 0.2)
                    ax_wABP.set_ylim((ymin, ymax))
                flag_autoscale = False
                
            # ------------------
            #  알람 범위 체크
            # ------------------
            hr_alarm = False
            spo2_alarm = False
            sbpdbp_alarm = False
            mbp_alarm = False
            
            if HR!=None and HR!=0:
                hr_na = False
                if HR<ALARM_THRESHOLDS['HR_LOW'] or HR>ALARM_THRESHOLDS['HR_HIGH']:
                    hr_alarm = True
            elif HR==None or HR==0:
                hr_na = True

            if SPO2!=None and SPO2!=0:
                spo2_na = False
                if SPO2<ALARM_THRESHOLDS['SPO2_LOW']:
                    spo2_alarm = True
            elif SPO2==None or SPO2==0:
                spo2_na = True

            if is_estiABP and len(buff_sBP)>0 and len(buff_dBP)>0 and len(buff_mBP)>0:
                asbp=sum(buff_sBP)/len(buff_sBP)
                adbp=sum(buff_dBP)/len(buff_dBP)
                ambp=sum(buff_mBP)/len(buff_mBP)

                # sBP/dBP 하나라도 벗어나면 -> sbpdbp_alarm
                if not spo2_na:
                    if asbp<ALARM_THRESHOLDS['SBP_LOW'] or asbp>ALARM_THRESHOLDS['SBP_HIGH'] or \
                       adbp<ALARM_THRESHOLDS['DBP_LOW'] or adbp>ALARM_THRESHOLDS['DBP_HIGH']:
                        sbpdbp_alarm=True

                    # mBP
                    if ambp<ALARM_THRESHOLDS['MBP_LOW'] or ambp>ALARM_THRESHOLDS['MBP_HIGH']:
                        mbp_alarm=True
                else:
                    sbpdbp_alarm=False
                    mbp_alarm=False
                    
            # 항목별 알람 상태 갱신
            hr_alarm_active   = hr_alarm
            spo2_alarm_active = spo2_alarm
            sdbp_alarm_active = sbpdbp_alarm
            mbp_alarm_active = mbp_alarm
            
            # 전체 알람
            overall_alarm = (hr_alarm or spo2_alarm or sdbp_alarm_active or mbp_alarm_active)

            # 알람On상태면 True전달, 아니면 False
            if alarm_enabled and overall_alarm:
                q_alarm_flag.put(True)
            else:
                q_alarm_flag.put(False)

        # ------------------
        #  글씨 깜빡임 (각 항목 알람 시)
        # ------------------
        now = time.time()
        if now - last_blink_time >= blink_interval:
            last_blink_time = now
            # HR blink
            if hr_alarm_active and not hr_na:
                blink_HR_state = not blink_HR_state
                txt_HR.set_color(color_HR_alarm if blink_HR_state else color_HR_normal)
            elif not hr_na:
                blink_HR_state = False
                txt_HR.set_color(color_HR_normal)
                    
            # SpO2 blink
            if spo2_alarm_active and not spo2_na:
                blink_SPO2_state = not blink_SPO2_state
                txt_SPO2.set_color(color_SPO2_alarm if blink_SPO2_state else color_SPO2_normal)
            elif not spo2_na:
                blink_SPO2_state = False
                txt_SPO2.set_color(color_SPO2_normal)

            # BP blink
            if sdbp_alarm_active and not spo2_na:
                blink_sdBP_state = not blink_sdBP_state
                color_now = color_BP_alarm if blink_sdBP_state else color_BP_normal
                txt_SBPDBP.set_color(color_now)
            elif not spo2_na:
                blink_sdBP_state = False
                txt_SBPDBP.set_color(color_BP_normal)

            # BP blink
            if mbp_alarm_active and not spo2_na:
                blink_mBP_state = not blink_mBP_state
                color_now = color_BP_alarm if blink_mBP_state else color_BP_normal
                txt_MAP.set_color(color_now)
            elif not spo2_na:
                blink_mBP_state = False
                txt_MAP.set_color(color_BP_normal)

        now_t = time.time() - base_time_ecg - 1.7
        ax_wECG.set_xlim(now_t - 5, now_t)
        now_t = time.time() - base_time_ppg - 2.0
        ax_wPPG.set_xlim(now_t - 5, now_t)
        ax_wABP.set_xlim(now_t - 5, now_t)

        # FPS
        execution_time = time.time() - t_pre
        t_pre = time.time()
        if execution_time == 0:
            execution_time = 0.01
        txt_FPS.set_text("{0:.0f} fps".format(1 / execution_time))

        fig.canvas.draw()
        root.update_idletasks()
        root.update()

    root.mainloop()



########################################################
# ABP 예측 프로세스
########################################################
def esti_ABP(q_ABPinput, q_ABPoutput, ABP_event):
    """
    ABP_model.tflite, wave_model.tflite 로딩하여
    PPG -> ABP 추정을 수행하는 프로세스.
    """
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
        return abpwave_interpreter.get_tensor(abpwave_output_details[0]['index'])

    def predict_value_PPG(normalized_ppg):
        input_data = np.expand_dims(np.array(normalized_ppg, dtype=np.float32), axis=0)
        abpvalue_interpreter.set_tensor(abpvalue_input_details[0]['index'], input_data)
        abpvalue_interpreter.invoke()
        return abpvalue_interpreter.get_tensor(abpvalue_output_details[0]['index'])

    abp_data_min = 20
    abp_data_max = 200
    abp_normalized_min = 0
    abp_normalized_max = 1
    cutoff_high = 12
    SAMPLING_RATE = 125

    def minmax_normalize(signal):
        mn, mx = np.min(signal), np.max(signal)
        if mn-mx == 0:
            return (signal - mn)/0.0000001, mn, mx
        else:
            return (signal - mn)/(mx - mn), mn, mx

    def apply_low_pass_filter(data, cutoff, sf, order=4, padlen=None):
        from scipy.signal import butter, filtfilt
        nyquist_freq = 0.5 * sf
        norm_cutoff = cutoff / nyquist_freq
        b, a = butter(order, norm_cutoff, btype='low', analog=False)
        if padlen is not None:
            return filtfilt(b, a, data, padlen=padlen)
        else:
            return filtfilt(b, a, data)

    def denormalize_data(normalized_data, data_min, data_max, norm_min, norm_max):
        return ((normalized_data - norm_min)*(data_max-data_min)/(norm_max-norm_min)) + data_min

    def check_none(arr):
        return np.any([element is None for element in arr])

    while True:
        if not q_ABPinput.empty():
            (wave_tECG, wave_ECG,
             wave_tPPG, wave_PPG,
             buff_HR, buff_SPO2, t_receive) = q_ABPinput.get_nowait()

            if ~check_none(wave_PPG):
                predict_result = predict_wave_PPG(wave_PPG)
                predict_result = np.array(predict_result).reshape(-1)
                predict_abp = denormalize_data(
                    predict_result,
                    abp_data_min, abp_data_max,
                    abp_normalized_min, abp_normalized_max
                )

                normalized_ppg, mn_val, mx_val = minmax_normalize(wave_PPG)
                results = predict_value_PPG(normalized_ppg)
                results = np.array(results).squeeze(axis=-1)
                predict_wave = results[0]
                abp_wave = apply_low_pass_filter(predict_wave, cutoff_high, SAMPLING_RATE)

                q_ABPoutput.put((wave_tECG, wave_ECG, wave_tPPG, wave_PPG,
                                 abp_wave, predict_abp,
                                 buff_HR, buff_SPO2,
                                 t_receive, True))
            else:
                q_ABPoutput.put((wave_tECG, wave_ECG, wave_tPPG, wave_PPG,
                                 0, 0,
                                 buff_HR, buff_SPO2,
                                 t_receive, False))
            ABP_event.clear()


#######################################
# 메인 프로세스
#######################################
if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    parser = argparse.ArgumentParser()
    opts = parser.parse_known_args()

    # ======================
    # 포트 선택 로직
    # ======================
    def update_loading_icon():
        global loading_label
        next_frame = next(loading_frames)  # 다음 프레임 가져오기
        loading_label.config(text=next_frame)  # 텍스트 업데이트
        root.after(300, update_loading_icon)  # 300ms 후 재호출

    def select_port(selected_port):
        global port_sel
        port_sel = selected_port
        print(f"선택한 COM 포트: {port_sel}")
        if root is not None:
            root.quit()

    def show_no_ports_message():
        tk.Label(root, text="사용 중인 COM 포트가 없습니다.").pack()
        tk.Button(root, text="확인", command=lambda: os._exit(0)).pack()

    def show_single_port_message(port):
        global port_sel
        port_sel, desc, hwid = port

    port_sel = None
    root = None

    def check_ports():
        ports = serial.tools.list_ports.comports()
        ports = [p for p in ports if not p.device.startswith('/dev/ttyAMA')]  # /dev/ttyAMA 제외
        if len(ports) == 1:
            show_single_port_message(ports[0])
            root.quit()
        else:
            root.after(1000, check_ports)

    # 포트 탐색
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    ports = [p for p in ports if not p.device.startswith('/dev/ttyAMA')]

    if len(ports) == 1:
        show_single_port_message(ports[0])
    else:
        root = tk.Tk()
        root.title("USB Port Connection")
        root.geometry("300x150")  # 창 크기 설정
        label = tk.Label(root, text="Waiting for USB connection...", font=("Arial", 12))
        label.pack(pady=20)
        loading_frames = cycle(["◐", "◓", "◑", "◒"])  # 간단한 원형 로딩 효과
        loading_label = tk.Label(root, text="", font=("Arial", 20))
        loading_label.pack(pady=10)
        update_loading_icon()
        check_ports()
        root.mainloop()

    if port_sel is not None:
        print(f"선택된 포트로 프로세스를 시작합니다: {port_sel}")
        tstream = PhilipsTelemetryStream(port=port_sel,
                                         values=["Pleth", 32*4, 'II', 64*8],
                                         polling_interval=0.05)
    else:
        print("포트가 선택되지 않았습니다.")
        os._exit(0)

    # QoS
    tstream.add_update_func(qos)

    # ============ 메인 프로세스에서 사용하는 큐/이벤트 ============
    stop_event = mp.Event()
    q_wave = mp.Queue()
    q_ABPoutput = mp.Queue()
    q_alarm_flag = mp.Queue()  # Plot 프로세스가 알람 필요여부(True/False) 전달

    # Plot 프로세스 시작
    p_plot = mp.Process(target=update_plot,
                        args=(q_wave, q_ABPoutput, stop_event, q_alarm_flag))
    p_plot.start()

    # ABP 추정 프로세스 시작
    ABP_event = mp.Event()
    q_ABPinput = mp.Queue()
    p_ABP = mp.Process(target=esti_ABP,
                       args=(q_ABPinput, q_ABPoutput, ABP_event))
    p_ABP.start()

    # TelemetryStream(Philips) 오픈
    tstream.open()

    # (메인 프로세스에서) PPG 입력버퍼
    from collections import deque
    buff_tPPG = deque([0]*1024, maxlen=1024)
    buff_PPG = deque([None]*1024, maxlen=1024)
    buff_HR = 0
    buff_SPO2 = 0

    last_poll = time.time()
    t_lastTrans = time.time()
    lastupdate_HR, lastupdate_SPO2 = 0, 0

    global alarm_playing, alarm_stop
    alarm_playing = False
    alarm_stop = False
    alarm_condition = False
    
    def watchdog():
        print("data communication error: program restart")
        print("buzzer off2")
        q_wave.put('done')
        p_plot.terminate()
        p_ABP.terminate()
        tstream.close()
        os._exit(1)

    try:
        while not stop_event.is_set():
            now = time.time()
            # Telemetry Poll
            if now > (last_poll + tstream.polling_interval):
                timer = threading.Timer(5, watchdog)
                timer.start()
                data = tstream.read(1, blocking=False)
                timer.cancel()

                if data:
                    t_receive = time.time()
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

                    temp = list(tstream.sampled_data.keys())
                    if ('II' in temp) and ('Pleth' in temp):
                        channel_ECG = tstream.sampled_data.get('II')
                        channel_PPG = tstream.sampled_data.get('Pleth')
                        if channel_ECG and channel_PPG:
                            ECG = channel_ECG.get('samples')
                            PPG = channel_PPG.get('samples')
                            if ECG and PPG:
                                # 업데이트된 PPG만 추출
                                idx_update = bisect.bisect_right(PPG.t, buff_tPPG[-1])
                                if idx_update < len(PPG.t):
                                    buff_tPPG.extend(PPG.t[idx_update:])
                                    buff_PPG.extend(PPG.y[idx_update:])
                                # 일정 주기(0.8초)에 한 번씩 ABP 추정 프로세스로 데이터 전달
                                if not ABP_event.is_set() and (time.time() - t_lastTrans > 0.8):
                                    ABP_event.set()
                                    q_ABPinput.put((ECG.t, ECG.y,
                                                    buff_tPPG, buff_PPG,
                                                    buff_HR, buff_SPO2,
                                                    t_receive))
                                    t_lastTrans = time.time()

                    last_poll = now

            # Plot 프로세스에서 알람 필요여부(True/False) 받기
            if not q_alarm_flag.empty():
                need_alarm = q_alarm_flag.get()
                if need_alarm and not alarm_condition:
                    alarm_condition = True
                    trigger_alarm()
                    print("[MAIN] Alarm ON")
                elif alarm_condition and (not need_alarm):
                    alarm_condition = False
                    stop_alarm()
                    print("[MAIN] Alarm OFF")

            time.sleep(0.01)

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
