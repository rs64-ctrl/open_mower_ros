"""
This launch file loads the parameters.

For v1 it maps the environment variables to the parameter server.
For v2 it loads the parameters into the parameter server via yaml file.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('open_mower')

    om_legacy_config_mode = os.environ.get('OM_LEGACY_CONFIG_MODE', 'False').lower() in ('true', '1', 'yes')
    mower = os.environ.get('MOWER', '')
    hardware_platform = os.environ.get('HARDWARE_PLATFORM', '2')
    esc_type = os.environ.get('ESC_TYPE', '')
    params_path = os.environ.get('PARAMS_PATH', '')

    actions = []

    if not om_legacy_config_mode:
        # OSv2 - params are stored as .yaml
        if mower == 'CUSTOM':
            # If it is a custom robot, load custom params
            custom_params_file = os.path.join(params_path, 'custom_params.yaml')
            if os.path.exists(custom_params_file):
                actions.append(
                    SetParametersFromFile(custom_params_file)
                )
        else:
            # v1 comms parameters for v1 platform (serial ports etc)
            if hardware_platform == '1':
                comms_general = os.path.join(
                    pkg_share, 'params', 'hardware_specific', mower, 'comms_general_params.yaml'
                )
                comms_esc = os.path.join(
                    pkg_share, 'params', 'hardware_specific', mower,
                    f'comms_{esc_type}_params.yaml'
                )
                comms_gps = os.path.join(
                    pkg_share, 'params', 'hardware_specific', mower, 'comms_gps_params.yaml'
                )
                # These would be loaded as global parameters; in ROS2, we pass them to specific nodes
                # For now, store the paths for other launch files to use
                actions.append(
                    SetParameter(name='_comms_general_params_file', value=comms_general)
                )
                actions.append(
                    SetParameter(name='_comms_esc_params_file', value=comms_esc)
                )
                actions.append(
                    SetParameter(name='_comms_gps_params_file', value=comms_gps)
                )

            # Default case: predefined mower model with user overrides
            defaults_file = os.path.join(pkg_share, 'params', 'openmower_defaults_v2.yaml')
            hw_params_file = os.path.join(
                pkg_share, 'params', 'hardware_specific', mower, 'params_v2.yaml'
            )
            user_params_file = os.path.join(params_path, 'mower_params.yaml')

            # Store parameter file paths for other nodes to load
            actions.append(
                SetParameter(name='_defaults_params_file', value=defaults_file)
            )

            # Load defaults YAML so parameters reach all nodes
            if os.path.exists(defaults_file):
                actions.append(SetParametersFromFile(defaults_file))

            if os.path.exists(hw_params_file):
                actions.append(
                    SetParameter(name='_hw_params_file', value=hw_params_file)
                )
                actions.append(SetParametersFromFile(hw_params_file))

            if os.path.exists(user_params_file):
                actions.append(
                    SetParameter(name='_user_params_file', value=user_params_file)
                )
                actions.append(SetParametersFromFile(user_params_file))
    else:
        # V1 hardware with v1 OS (params are stored as environment variables)
        # Build a dictionary of all parameters from environment variables
        legacy_params = {}

        if mower != 'CUSTOM':
            # comms_general_params and comms_esc_params paths stored for node loading
            pass

        # ll/services/diff_drive params
        om_wheel_ticks = os.environ.get('OM_WHEEL_TICKS_PER_M', '0')
        om_wheel_distance = os.environ.get('OM_WHEEL_DISTANCE_M', '0')
        legacy_params['ll.services.diff_drive.ticks_per_m'] = om_wheel_ticks
        legacy_params['ll.services.diff_drive.wheel_distance_m'] = om_wheel_distance

        # Sound params
        legacy_params['ll.services.sound.dfp_is_5v'] = os.environ.get('OM_DFP_IS_5V', 'False')
        legacy_params['ll.services.sound.language'] = os.environ.get('OM_LANGUAGE', 'en')
        legacy_params['ll.services.sound.volume'] = os.environ.get('OM_VOLUME', '-1')
        legacy_params['ll.services.sound.background_sounds'] = os.environ.get('OM_BACKGROUND_SOUNDS', 'False')

        # XESC ports
        xesc_left = os.environ.get('OM_XESC_LEFT_PORT', '')
        xesc_right = os.environ.get('OM_XESC_RIGHT_PORT', '')
        xesc_mower = os.environ.get('OM_XESC_MOWER_PORT', '')
        if xesc_left:
            legacy_params['ll.services.diff_drive.left_xesc.serial_port'] = xesc_left
        if xesc_right:
            legacy_params['ll.services.diff_drive.right_xesc.serial_port'] = xesc_right
        if xesc_mower:
            legacy_params['ll.services.diff_drive.mower_xesc.serial_port'] = xesc_mower

        # GPS params
        gps_baudrate = os.environ.get('OM_GPS_BAUDRATE', '')
        gps_port = os.environ.get('OM_GPS_PORT', '')
        gps_device_type = os.environ.get('OM_GPS_DEVICE_TYPE', 'serial')
        if gps_baudrate:
            legacy_params['ll.services.gps.baudrate'] = gps_baudrate
        if gps_port:
            legacy_params['ll.services.gps.serial_port'] = gps_port
        legacy_params['ll.services.gps.device_type'] = gps_device_type
        if gps_device_type == 'tcp':
            legacy_params['ll.services.gps.tcp_host'] = os.environ.get('OM_GPS_TCP_HOSTNAME', '127.0.0.1')
            legacy_params['ll.services.gps.tcp_port'] = os.environ.get('OM_GPS_TCP_PORT', '2102')

        use_relative = os.environ.get('OM_USE_RELATIVE_POSITION', 'False').lower() in ('true', '1', 'yes')
        if use_relative:
            legacy_params['ll.services.gps.mode'] = 'relative'
        else:
            legacy_params['ll.services.gps.mode'] = 'absolute'

        legacy_params['ll.services.gps.protocol'] = os.environ.get('OM_GPS_PROTOCOL', 'UBX')

        if not use_relative:
            legacy_params['ll.services.gps.datum_lat'] = os.environ.get('OM_DATUM_LAT', '0')
            legacy_params['ll.services.gps.datum_long'] = os.environ.get('OM_DATUM_LONG', '0')
            legacy_params['ll.services.gps.datum_height'] = '0'

        # xbot_positioning
        legacy_params['xbot_positioning.max_gps_accuracy'] = '0.2'
        legacy_params['xbot_positioning.antenna_offset_x'] = os.environ.get('OM_ANTENNA_OFFSET_X', '0')
        legacy_params['xbot_positioning.antenna_offset_y'] = os.environ.get('OM_ANTENNA_OFFSET_Y', '0')
        legacy_params['xbot_positioning.debug'] = 'false'

        # ntrip_client
        legacy_params['ntrip_client.host'] = os.environ.get('OM_NTRIP_HOSTNAME', '')
        legacy_params['ntrip_client.port'] = os.environ.get('OM_NTRIP_PORT', '')
        legacy_params['ntrip_client.mountpoint'] = os.environ.get('OM_NTRIP_ENDPOINT', '')
        legacy_params['ntrip_client.username'] = os.environ.get('OM_NTRIP_USER', '')
        legacy_params['ntrip_client.password'] = os.environ.get('OM_NTRIP_PASSWORD', '')
        legacy_params['ntrip_client.reconnect_attempt_wait_seconds'] = os.environ.get(
            'OM_NTRIP_RECONNECT_WAIT_SECONDS', '5'
        )
        legacy_params['ntrip_client.reconnect_attempt_max'] = os.environ.get('OM_NTRIP_RECONNECT_MAX', '99999')

        # mower_logic
        battery_empty = os.environ.get('OM_BATTERY_EMPTY_VOLTAGE', '0')
        battery_critical = os.environ.get('OM_BATTERY_CRITICAL_VOLTAGE', '')
        legacy_params['mower_logic.automatic_mode'] = os.environ.get('OM_AUTOMATIC_MODE', '0')
        legacy_params['mower_logic.docking_distance'] = os.environ.get('OM_DOCKING_DISTANCE', '0')
        legacy_params['mower_logic.docking_approach_distance'] = os.environ.get('OM_DOCKING_APPROACH_DISTANCE', '1.5')
        legacy_params['mower_logic.docking_extra_time'] = os.environ.get('OM_DOCKING_EXTRA_TIME', '0')
        legacy_params['mower_logic.docking_retry_count'] = os.environ.get('OM_DOCKING_RETRY_COUNT', '4')
        legacy_params['mower_logic.docking_redock'] = os.environ.get('OM_DOCKING_REDOCK', 'False')
        legacy_params['mower_logic.undock_distance'] = os.environ.get('OM_UNDOCK_DISTANCE', '0')
        legacy_params['mower_logic.perimeter_signal'] = os.environ.get('OM_PERIMETER_SIGNAL', '')
        legacy_params['mower_logic.tool_width'] = os.environ.get('OM_TOOL_WIDTH', '0')
        legacy_params['mower_logic.enable_mower'] = os.environ.get('OM_ENABLE_MOWER', 'False')
        legacy_params['ll.services.power.battery_empty_voltage'] = battery_empty
        if battery_critical:
            legacy_params['ll.services.power.battery_critical_voltage'] = battery_critical
        else:
            legacy_params['ll.services.power.battery_critical_voltage'] = battery_empty
        legacy_params['ll.services.power.battery_full_voltage'] = os.environ.get('OM_BATTERY_FULL_VOLTAGE', '0')
        legacy_params['ll.services.power.battery_critical_high_voltage'] = os.environ.get(
            'OM_BATTERY_CRITICAL_HIGH_VOLTAGE', '-1'
        )
        legacy_params['ll.services.power.charge_critical_high_voltage'] = os.environ.get(
            'OM_CHARGE_CRITICAL_HIGH_VOLTAGE', '-1'
        )
        legacy_params['ll.services.power.charge_critical_high_current'] = os.environ.get(
            'OM_CHARGE_CRITICAL_HIGH_CURRENT', '-1'
        )
        legacy_params['mower_logic.outline_count'] = os.environ.get('OM_OUTLINE_COUNT', '0')
        legacy_params['mower_logic.outline_overlap_count'] = os.environ.get('OM_OUTLINE_OVERLAP_COUNT', '0')
        legacy_params['mower_logic.outline_offset'] = os.environ.get('OM_OUTLINE_OFFSET', '0')
        legacy_params['mower_logic.mow_angle_offset'] = os.environ.get('OM_MOWING_ANGLE_OFFSET', '0')
        legacy_params['mower_logic.mow_angle_offset_is_absolute'] = os.environ.get(
            'OM_MOWING_ANGLE_OFFSET_IS_ABSOLUTE', 'False'
        )
        legacy_params['mower_logic.mow_angle_increment'] = os.environ.get('OM_MOWING_ANGLE_INCREMENT', '0')
        legacy_params['mower_logic.motor_hot_temperature'] = os.environ.get('OM_MOWING_MOTOR_TEMP_HIGH', '0')
        legacy_params['mower_logic.motor_cold_temperature'] = os.environ.get('OM_MOWING_MOTOR_TEMP_LOW', '0')
        legacy_params['mower_logic.gps_wait_time'] = os.environ.get('OM_GPS_WAIT_TIME_SEC', '10.0')
        legacy_params['mower_logic.gps_timeout'] = os.environ.get('OM_GPS_TIMEOUT_SEC', '10.0')
        legacy_params['mower_logic.rain_mode'] = os.environ.get('OM_RAIN_MODE', '0')
        legacy_params['mower_logic.rain_delay_minutes'] = os.environ.get('OM_RAIN_DELAY_MINUTES', '30')
        legacy_params['mower_logic.rain_check_seconds'] = os.environ.get('OM_RAIN_CHECK_SECONDS', '20')
        legacy_params['mower_logic.cu_rain_threshold'] = os.environ.get('OM_CU_RAIN_THRESHOLD', '-1')
        legacy_params['mower_logic.undock_angled_distance'] = os.environ.get('OM_UNDOCK_ANGLED_DISTANCE', '0.0')
        legacy_params['mower_logic.undock_angle'] = os.environ.get('OM_UNDOCK_ANGLE', '0.0')
        legacy_params['mower_logic.undock_fixed_angle'] = os.environ.get('OM_UNDOCK_FIXED_ANGLE', 'True')
        legacy_params['mower_logic.undock_use_curve'] = os.environ.get('OM_UNDOCK_USE_CURVE', 'True')
        legacy_params['mower_logic.docking_waiting_time'] = os.environ.get('OM_DOCKING_WAIT_TIME', '0.0')
        legacy_params['mower_logic.undocking_waiting_time'] = os.environ.get('OM_UNDOCKING_WAIT_TIME', '0.0')
        legacy_params['mower_logic.emergency_lift_period'] = os.environ.get('OM_EMERGENCY_LIFT_PERIOD', '-1')
        legacy_params['mower_logic.emergency_tilt_period'] = os.environ.get('OM_EMERGENCY_TILT_PERIOD', '-1')
        legacy_params['mower_logic.emergency_input_config'] = os.environ.get('OM_EMERGENCY_INPUT_CONFIG', '')
        legacy_params['mower_logic.ignore_charging_current'] = os.environ.get('OM_IGNORE_CHARGING_CURRENT', 'False')
        legacy_params['mower_logic.shutdown_esc_max_pitch'] = os.environ.get('OM_SHUTDOWN_ESC_MAX_PITCH', '0')

        # xbot_monitoring
        legacy_params['xbot_monitoring.external_mqtt_enable'] = os.environ.get('OM_MQTT_ENABLE', 'False')
        legacy_params['xbot_monitoring.external_mqtt_hostname'] = os.environ.get('OM_MQTT_HOSTNAME', '')
        legacy_params['xbot_monitoring.external_mqtt_port'] = os.environ.get('OM_MQTT_PORT', '')
        legacy_params['xbot_monitoring.external_mqtt_username'] = os.environ.get('OM_MQTT_USER', '')
        legacy_params['xbot_monitoring.external_mqtt_password'] = os.environ.get('OM_MQTT_PASSWORD', '')
        legacy_params['xbot_monitoring.external_mqtt_topic_prefix'] = os.environ.get('OM_MQTT_TOPIC_PREFIX', '')
        legacy_params['xbot_monitoring.software_version'] = os.environ.get('OM_SOFTWARE_VERSION', '')

        # heatmap_generator
        legacy_params['sensor_ids'] = os.environ.get('OM_HEATMAP_SENSOR_IDS', '')

        # Store all legacy params as SetParameter actions
        for key, value in legacy_params.items():
            actions.append(SetParameter(name=key, value=value))

    return LaunchDescription(actions)
