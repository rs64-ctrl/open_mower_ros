#!/usr/bin/env python3

import sys
import importlib

import rclpy

from ntrip_client.ntrip_ros_base import NTRIPRosBase
from ntrip_client.ntrip_serial_device import NTRIPSerialDevice

# Try to import a couple different types of RTCM messages
_MAVROS_MSGS_NAME = "mavros_msgs"
_RTCM_MSGS_NAME = "rtcm_msgs"
have_mavros_msgs = False
have_rtcm_msgs = False
if importlib.util.find_spec(_MAVROS_MSGS_NAME) is not None:
  have_mavros_msgs = True
  from mavros_msgs.msg import RTCM as mavros_msgs_RTCM
if importlib.util.find_spec(_RTCM_MSGS_NAME) is not None:
  have_rtcm_msgs = True
  from rtcm_msgs.msg import Message as rtcm_msgs_RTCM

class NTRIPRos(NTRIPRosBase):
  def __init__(self):
    # Init the node
    super().__init__('ntrip_client')

    # Read some mandatory config
    self.declare_parameter('port', '/dev/ttyACM0')
    self.declare_parameter('baudrate', 115200)
    port = self.get_parameter('port').get_parameter_value().string_value
    baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

    # Initialize the client
    self._client = NTRIPSerialDevice(
      port=port,
      baudrate=baudrate,
      logerr=self.get_logger().error,
      logwarn=self.get_logger().warning,
      loginfo=self.get_logger().info,
      logdebug=self.get_logger().debug
    )

    # Set parameters on the client
    self._client.nmea_parser.nmea_max_length = self._nmea_max_length
    self._client.nmea_parser.nmea_min_length = self._nmea_min_length
    self._client.reconnect_attempt_max = self._reconnect_attempt_max
    self._client.reconnect_attempt_wait_seconds = self._reconnect_attempt_wait_seconds
    self._client.reconnect_backoff_base = self._reconnect_backoff_base
    self._client.reconnect_backoff_max_seconds = self._reconnect_backoff_max_seconds


def main(args=None):
  rclpy.init(args=args)
  ntrip_ros = NTRIPRos()
  rc = ntrip_ros.run()
  ntrip_ros.destroy_node()
  rclpy.shutdown()
  sys.exit(rc)

if __name__ == '__main__':
  main()
