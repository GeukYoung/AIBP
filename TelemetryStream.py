from __future__ import unicode_literals, division

import argparse
import logging
import logging.handlers
import time
import json
import datetime
import numpy as np

class SampledDataBuffer(object):
    # This is a fixed-length double queue for time/value pairs s.t. f(t)=y
    # Once initialized, it can be updated with a single time point and a set of values
    # taken at a given frequency.
    # t is re-evaluated relative to the initialization time

    def __init__(self, freq, dur):
        self.freq = freq
        self.dur = dur
        self.y = np.zeros(self.freq*self.dur)
        self.t = np.zeros(self.freq*self.dur)
        self.start_time = datetime.datetime.now()
        self.t1 = None
        self.dropped_packets = 0

    def rolling_append(self, _t0, values):

        if values is None:
            return

        # Convert t0 to seconds since start_time
        t0 = (_t0 - self.start_time).total_seconds()
        if not self.t1:
            self.t1 = t0
        t0 -= self.t1

        length = values.size or 1  # For scalar
        if length == 1:
            times = t0
        else:
            sec_offset = length/self.freq
            times = np.linspace(t0, t0+sec_offset, length)

        self.y = np.roll(self.y, -length)
        self.y[-length:] = values
        self.t = np.roll(self.t, -length)
        self.t[-length:] = times

class TelemetryStream(object):
    # This is an abstract class and/or factory that provides a consistent interface across
    # vendors and devices.

    def __init__(self, *args, **kwargs):
        # Setup a specialized output logger
        self.logger = logging.getLogger()
        self.update_funcs = []
        self.polling_interval = kwargs.get('polling_interval', 0.25)
        self.sampled_data_dur = kwargs.get('sampled_data_dur', 7)
        self.sampled_data = {}

        sampled_data_args = kwargs.get('values')
        if sampled_data_args:
            for key, freq in zip(sampled_data_args[0::2], sampled_data_args[1::2]):
                self.sampled_data[key] = {'freq': int(freq),
                                          'samples': SampledDataBuffer(int(freq),
                                                                       self.sampled_data_dur)}

    def update_sampled_data(self, data):
        if not data:
            return

        for key, value in data.items():
            if key in self.sampled_data.keys():
                t = data['timestamp']
                y = data[key]
                self.sampled_data[key]['samples'].rolling_append(t,y)
                    
    def __del__(self):
        # Note that logging may no longer exist by here
        print("Tearing down connection object")
        self.close()

    def add_update_func(self, f):
        self.update_funcs.append(f)

    def run(self, blocking=False):
        # Create a main loop that just echoes the results to the loggers
        self.open()
        while 1:
            self.read(1, blocking=blocking) # 데이터 수집 파트 by JG
            time.sleep(self.polling_interval)

    def open(self, *args, **kwargs):
        raise NotImplementedError

    def close(self, *args, **kwargs):
        raise NotImplementedError

    def read(self, *args, **kwargs):
        # Read should echo data to self.logger at "info" level
        raise NotImplementedError


class TelemetryEncoder(json.JSONEncoder):
    def default(self, o):
        # Deal with datetime
        if isinstance(o, datetime.datetime):
            return o.isoformat()
        # Deal with numpy
        if type(o).__module__ == np.__name__:
            return o.tolist()
        return json.JSONEncoder.default(self, o)

# def configure_parser(parser):
#     parser.add_argument('-b', '--binary', help="Name of an hdf5 file for binary logging")
#     parser.add_argument('-f', '--file', help="Name of a text file for event logging")
#     parser.add_argument('-ht', '--host_time', help="Include host time in file outputs", action='store_true')
#     parser.add_argument('-s', '--splunk', help="Name of a Splunk index for event logging")
#     parser.add_argument('-g', '--gui', help="Display a graphic user interface, e.g., 'SimpleStripchart'")
#     # Default for PL203 usb to serial device
#     parser.add_argument('-p', '--port', help="Device port (or 'sample')", default="/dev/cu.usbserial")
#     parser.add_argument('--values', nargs="+",
#                         help="List of paired value names and frequencies to monitor, e.g. 'Pleth 128 ECG 256'",
#                         default=['Pleth', 128, 'ECG', 256])
#     return parser


# def parse_args():
#     # Creates an options array from the command-line arguments

#     parser = argparse.ArgumentParser(description=__description__)
#     parser.add_argument('-V', '--version',
#                         action='version',
#                         version='%(prog)s (version ' + __hash__ + ')')
#     parser = configure_parser(parser)
#     _opts = parser.parse_args()
#     return _opts
