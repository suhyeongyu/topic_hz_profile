# Copyright (c) 2008, Willow Garage, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This file is originally from:
# https://github.com/ros/ros_comm/blob/6e5016f4b2266d8a60c9a1e163c4928b8fc7115e/tools/rostopic/src/rostopic/__init__.py
#
# MODIFIED: Added CLI options to configure QoS for the hz subscription:
#   --qos-depth        (default: 100, was effectively 5)
#   --qos-reliability  (best_effort | reliable | system_default)
#   --qos-durability   (volatile | transient_local | system_default)
#   --qos-history      (keep_last | keep_all | system_default)
# The original code hardcoded qos_profile_sensor_data, which has
# depth=5 + BEST_EFFORT + VOLATILE + KEEP_LAST, and that caused heavy message
# drops at high publish rates. The defaults below preserve the original
# behavior except that depth is bumped from 5 to 100.

from argparse import ArgumentTypeError
from collections import defaultdict

import functools
import math
import threading

import rclpy

from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from ros2cli.node.direct import add_arguments as add_direct_node_arguments
from ros2cli.node.direct import DirectNode
from ros2topic.api import get_msg_class
from ros2topic.api import TopicNameCompleter
from ros2topic.verb import VerbExtension

DEFAULT_WINDOW_SIZE = 10000
DEFAULT_QOS_DEPTH = 100  # was effectively 5 (from qos_profile_sensor_data)
DEFAULT_QOS_RELIABILITY = 'best_effort'
DEFAULT_QOS_DURABILITY = 'volatile'
DEFAULT_QOS_HISTORY = 'keep_last'

_RELIABILITY_MAP = {
    'best_effort':    QoSReliabilityPolicy.BEST_EFFORT,
    'reliable':       QoSReliabilityPolicy.RELIABLE,
    'system_default': QoSReliabilityPolicy.SYSTEM_DEFAULT,
}
_DURABILITY_MAP = {
    'volatile':        QoSDurabilityPolicy.VOLATILE,
    'transient_local': QoSDurabilityPolicy.TRANSIENT_LOCAL,
    'system_default':  QoSDurabilityPolicy.SYSTEM_DEFAULT,
}
_HISTORY_MAP = {
    'keep_last':      QoSHistoryPolicy.KEEP_LAST,
    'keep_all':       QoSHistoryPolicy.KEEP_ALL,
    'system_default': QoSHistoryPolicy.SYSTEM_DEFAULT,
}


def positive_int(string):
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value <= 0:
        raise ArgumentTypeError('value must be a positive integer')
    return value


def _build_qos_profile(depth, reliability, durability, history):
    """Assemble a QoSProfile from CLI string args."""
    return QoSProfile(
        depth=depth,
        reliability=_RELIABILITY_MAP[reliability],
        durability=_DURABILITY_MAP[durability],
        history=_HISTORY_MAP[history],
    )


class HzVerb(VerbExtension):
    """Print the average receiving rate to screen."""

    def add_arguments(self, parser, cli_name):
        parser.description = (
            'Print the average receiving rate to screen.\n\n'
            'note:\n'
            '  This rate reflects the receiving rate on subscription, '
            'which might be affected by platform resources and QoS configuration, '
            "and may not exactly match the publisher's rate."
        )
        arg = parser.add_argument(
            'topic_name',
            help="Name of the ROS topic to listen to (e.g. '/chatter')")
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')
        parser.add_argument(
            '--window', '-w',
            dest='window_size', type=positive_int, default=DEFAULT_WINDOW_SIZE,
            help='window size, in # of messages, for calculating rate '
                 '(default: %d)' % DEFAULT_WINDOW_SIZE, metavar='WINDOW')
        parser.add_argument(
            '--filter',
            dest='filter_expr', default=None,
            help='only measure messages matching the specified Python expression', metavar='EXPR')
        parser.add_argument(
            '--wall-time',
            dest='use_wtime', default=False, action='store_true',
            help='calculates rate using wall time which can be helpful'
                 ' when clock is not published during simulation')
        # --- PATCH: expose QoS settings ---
        parser.add_argument(
            '--qos-depth',
            dest='qos_depth', type=positive_int, default=DEFAULT_QOS_DEPTH,
            help='DDS history depth for the hz subscription '
                 '(default: %d). The original hz used qos_profile_sensor_data '
                 'with depth=5, which caused message drops at high rates.'
                 % DEFAULT_QOS_DEPTH)
        parser.add_argument(
            '--qos-reliability',
            dest='qos_reliability',
            choices=('best_effort', 'reliable', 'system_default'),
            default=DEFAULT_QOS_RELIABILITY,
            help='QoS reliability policy (default: %s). Must be compatible '
                 'with the publisher. A RELIABLE subscriber cannot receive '
                 'from a BEST_EFFORT publisher.' % DEFAULT_QOS_RELIABILITY)
        parser.add_argument(
            '--qos-durability',
            dest='qos_durability',
            choices=('volatile', 'transient_local', 'system_default'),
            default=DEFAULT_QOS_DURABILITY,
            help='QoS durability policy (default: %s).'
                 % DEFAULT_QOS_DURABILITY)
        parser.add_argument(
            '--qos-history',
            dest='qos_history',
            choices=('keep_last', 'keep_all', 'system_default'),
            default=DEFAULT_QOS_HISTORY,
            help='QoS history policy (default: %s). depth is only honored '
                 'with keep_last.' % DEFAULT_QOS_HISTORY)
        add_direct_node_arguments(parser)

    def main(self, *, args):
        return main(args)


def main(args):
    topic = args.topic_name
    if args.filter_expr:
        def expr_eval(expr):
            def eval_fn(m):
                return eval(expr)
            return eval_fn
        filter_expr = expr_eval(args.filter_expr)
    else:
        filter_expr = None

    with DirectNode(args) as node:
        _rostopic_hz(
            node.node, topic,
            window_size=args.window_size,
            filter_expr=filter_expr,
            use_wtime=args.use_wtime,
            qos_depth=args.qos_depth,
            qos_reliability=args.qos_reliability,
            qos_durability=args.qos_durability,
            qos_history=args.qos_history,
        )


class ROSTopicHz(object):
    """ROSTopicHz receives messages for a topic and computes frequency."""

    def __init__(self, node, window_size, filter_expr=None, use_wtime=False):
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.msg_t0 = -1
        self.msg_tn = 0
        self.times = []
        self._last_printed_tn = defaultdict(int)
        self._msg_t0 = defaultdict(lambda: -1)
        self._msg_tn = defaultdict(int)
        self._times = defaultdict(list)
        self.filter_expr = filter_expr
        self.use_wtime = use_wtime

        self.window_size = window_size

        # Clock that has support for ROS time.
        self._clock = node.get_clock()

    def get_last_printed_tn(self, topic=None):
        if topic is None:
            return self.last_printed_tn
        return self._last_printed_tn[topic]

    def set_last_printed_tn(self, value, topic=None):
        if topic is None:
            self.last_printed_tn = value
        self._last_printed_tn[topic] = value

    def get_msg_t0(self, topic=None):
        if topic is None:
            return self.msg_t0
        return self._msg_t0[topic]

    def set_msg_t0(self, value, topic=None):
        if topic is None:
            self.msg_t0 = value
        self._msg_t0[topic] = value

    def get_msg_tn(self, topic=None):
        if topic is None:
            return self.msg_tn
        return self._msg_tn[topic]

    def set_msg_tn(self, value, topic=None):
        if topic is None:
            self.msg_tn = value
        self._msg_tn[topic] = value

    def get_times(self, topic=None):
        if topic is None:
            return self.times
        return self._times[topic]

    def set_times(self, value, topic=None):
        if topic is None:
            self.times = value
        self._times[topic] = value

    def callback_hz(self, m, topic=None):
        """
        Calculate interval time.

        :param m: Message instance
        :param topic: Topic name
        """
        # ignore messages that don't match filter
        if self.filter_expr is not None and not self.filter_expr(m):
            return
        with self.lock:
            # Uses ROS time as the default time source and Walltime only if requested
            curr_rostime = self._clock.now() if not self.use_wtime else \
                Clock(clock_type=ClockType.SYSTEM_TIME).now()

            # time reset
            if curr_rostime.nanoseconds == 0:
                if len(self.get_times(topic=topic)) > 0:
                    print('time has reset, resetting counters')
                    self.set_times([], topic=topic)
                return

            curr = curr_rostime.nanoseconds
            msg_t0 = self.get_msg_t0(topic=topic)
            if msg_t0 < 0 or msg_t0 > curr:
                self.set_msg_t0(curr, topic=topic)
                self.set_msg_tn(curr, topic=topic)
                self.set_times([], topic=topic)
            else:
                self.get_times(topic=topic).append(curr - self.get_msg_tn(topic=topic))
                self.set_msg_tn(curr, topic=topic)

            if len(self.get_times(topic=topic)) > self.window_size:
                self.get_times(topic=topic).pop(0)

    def get_hz(self, topic=None):
        """
        Calculate the average publising rate.

        :param topic: topic name, ``list`` of ``str``
        :returns: tuple of stat results
            (rate, min_delta, max_delta, standard deviation, window number)
            None when waiting for the first message or there is no new one
        """
        if not self.get_times(topic=topic):
            return
        elif self.get_last_printed_tn(topic=topic) == 0:
            self.set_last_printed_tn(self.get_msg_tn(topic=topic), topic=topic)
            return
        elif self.get_msg_tn(topic=topic) < self.get_last_printed_tn(topic=topic) + 1e9:
            return
        with self.lock:
            # Get frequency every one minute
            times = self.get_times(topic=topic)
            n = len(times)
            mean = sum(times) / n
            rate = 1. / mean if mean > 0. else 0

            # std dev
            std_dev = math.sqrt(sum((x - mean)**2 for x in times) / n)

            # min and max
            max_delta = max(times)
            min_delta = min(times)

            self.set_last_printed_tn(self.get_msg_tn(topic=topic), topic=topic)

        return rate, min_delta, max_delta, std_dev, n

    def print_hz(self, topic=None):
        """Print the average publishing rate to screen."""
        ret = self.get_hz(topic)
        if ret is None:
            return
        rate, min_delta, max_delta, std_dev, window = ret
        print('average rate: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s'
              % (rate * 1e9, min_delta * 1e-9, max_delta * 1e-9, std_dev * 1e-9, window))
        return


def _rostopic_hz(node, topic,
                 window_size=DEFAULT_WINDOW_SIZE,
                 filter_expr=None,
                 use_wtime=False,
                 qos_depth=DEFAULT_QOS_DEPTH,
                 qos_reliability=DEFAULT_QOS_RELIABILITY,
                 qos_durability=DEFAULT_QOS_DURABILITY,
                 qos_history=DEFAULT_QOS_HISTORY):
    """
    Periodically print the publishing rate of a topic to console until shutdown.

    :param topic: topic name, ``list`` of ``str``
    :param window_size: number of messages to average over, -1 for infinite, ``int``
    :param filter_expr: Python filter expression that is called with m, the message instance
    :param qos_depth: DDS history depth
    :param qos_reliability: 'best_effort' | 'reliable' | 'system_default'
    :param qos_durability:  'volatile' | 'transient_local' | 'system_default'
    :param qos_history:     'keep_last' | 'keep_all' | 'system_default'
    """
    # pause hz until topic is published
    msg_class = get_msg_class(
        node, topic, blocking=True, include_hidden_topics=True)

    if msg_class is None:
        node.destroy_node()
        return

    # --- PATCH: build a user-configurable QoS profile. The defaults match
    # the old behavior (BEST_EFFORT + VOLATILE + KEEP_LAST) except that
    # depth is raised from 5 to 100 to avoid drops at high rates.
    qos_profile = _build_qos_profile(
        qos_depth, qos_reliability, qos_durability, qos_history)
    print('[hz] QoS: reliability=%s durability=%s history=%s depth=%d'
          % (qos_reliability, qos_durability, qos_history, qos_depth))

    rt = ROSTopicHz(node, window_size, filter_expr=filter_expr, use_wtime=use_wtime)
    node.create_subscription(
        msg_class,
        topic,
        functools.partial(rt.callback_hz, topic=topic),
        qos_profile)

    while rclpy.ok():
        rclpy.spin_once(node)
        rt.print_hz(topic)

    node.destroy_node()
    rclpy.shutdown()
