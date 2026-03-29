#!/usr/bin/env python3

import sys
import importlib

import rclpy

from ntrip_client.ntrip_ros_base import NTRIPRosBase
from ntrip_client.ntrip_client import NTRIPClient

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
    self.declare_parameter('host', '127.0.0.1')
    self.declare_parameter('port', '2101')
    self.declare_parameter('mountpoint', 'mount')
    host = self.get_parameter('host').get_parameter_value().string_value
    port = self.get_parameter('port').get_parameter_value().string_value
    mountpoint = self.get_parameter('mountpoint').get_parameter_value().string_value

    # Optionally get the ntrip version from the launch file
    self.declare_parameter('ntrip_version', '')
    ntrip_version = self.get_parameter('ntrip_version').get_parameter_value().string_value
    if ntrip_version == '':
      ntrip_version = None

    # If we were asked to authenticate, read the username and password
    username = None
    password = None
    self.declare_parameter('authenticate', False)
    self.declare_parameter('username', '')
    self.declare_parameter('password', '')
    if self.get_parameter('authenticate').get_parameter_value().bool_value:
      username = self.get_parameter('username').get_parameter_value().string_value
      password = self.get_parameter('password').get_parameter_value().string_value
      if username == '':
        self.get_logger().error(
          'Requested to authenticate, but param "username" was not set')
        sys.exit(1)
      if password == '':
        self.get_logger().error(
          'Requested to authenticate, but param "password" was not set')
        sys.exit(1)

    # Initialize the client
    self._client = NTRIPClient(
      host=host,
      port=port,
      mountpoint=mountpoint,
      ntrip_version=ntrip_version,
      username=username,
      password=password,
      logerr=self.get_logger().error,
      logwarn=self.get_logger().warning,
      loginfo=self.get_logger().info,
      logdebug=self.get_logger().debug
    )

    # Get some SSL parameters for the NTRIP client
    self.declare_parameter('ssl', False)
    self.declare_parameter('cert', '')
    self.declare_parameter('key', '')
    self.declare_parameter('ca_cert', '')
    self._client.ssl = self.get_parameter('ssl').get_parameter_value().bool_value
    cert_val = self.get_parameter('cert').get_parameter_value().string_value
    key_val = self.get_parameter('key').get_parameter_value().string_value
    ca_cert_val = self.get_parameter('ca_cert').get_parameter_value().string_value
    self._client.cert = cert_val if cert_val != '' else None
    self._client.key = key_val if key_val != '' else None
    self._client.ca_cert = ca_cert_val if ca_cert_val != '' else None

    # Set parameters on the client
    self._client.nmea_parser.nmea_max_length = self._nmea_max_length
    self._client.nmea_parser.nmea_min_length = self._nmea_min_length
    self._client.reconnect_attempt_max = self._reconnect_attempt_max
    self._client.reconnect_attempt_wait_seconds = self._reconnect_attempt_wait_seconds
    self._client.reconnect_backoff_base = self._reconnect_backoff_base
    self._client.reconnect_backoff_max_seconds = self._reconnect_backoff_max_seconds

    self.declare_parameter('rtcm_timeout_seconds', NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS)
    self._client.rtcm_timeout_seconds = self.get_parameter('rtcm_timeout_seconds').get_parameter_value().integer_value


def main(args=None):
  rclpy.init(args=args)
  ntrip_ros = NTRIPRos()
  rc = ntrip_ros.run()
  ntrip_ros.destroy_node()
  rclpy.shutdown()
  sys.exit(rc)

if __name__ == '__main__':
  main()
