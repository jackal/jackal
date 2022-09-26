#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2022 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)


from diagnostic_msgs.msg import DiagnosticStatus

from diagnostic_updater import FrequencyStatusParam, HeaderlessTopicDiagnostic, Updater

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from jackal_msgs.msg import Status, StopStatus, Power, Feedback

from nmea_msgs.msg import Sentence

from sensor_msgs.msg import Imu, MagneticField

from std_msgs.msg import Bool


class JackalDiagnosticsUpdater(Node):

    def __init__(self):
        super().__init__('jackal_diagnostics')

        # Create updater
        self.updater = Updater(self)
        self.updater.setHardwareID('Jackal')

        # Subscribe to topics
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.mag_sub = self.create_subscription(
            MagneticField,
            'imu/mag',
            self.mag_callback,
            qos_profile_sensor_data
        )

        self.status_mcu_sub = self.create_subscription(
            Status,
            'status/mcu',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.status_stop_sub = self.create_subscription(
            StopStatus,
            'status/stop',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.status_power_sub = self.create_subscription(
            Power,
            'status/power',
            self.power_callback,
            qos_profile_sensor_data
        )

        self.stop_engaged_sub = self.create_subscription(
            Bool,
            'stop_engaged',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.feedback_sub = self.create_subscription(
            Feedback,
            'feedback',
            self.feedback_callback,
            qos_profile_sensor_data
        )

        self.navsat_sub = self.create_subscription(
            Sentence,
            'navsat/nmea_sentence',
            self.navsat_callback,
            qos_profile_sensor_data
        )

        # Initial values
        self.measured_voltages = [0.0] * 3
        self.measured_currents = [0.0] * 4

        self.stop_power_status = False
        self.external_stop_present = False

        self.imu_freq_bounds = {'min': 45, 'max': 55}
        self.feedback_freq_bounds = {'min': 45, 'max': 55}
        self.navsat_freq_bounds = {'min': 8, 'max': 12}

        # Add Diagnostic status
        self.updater.add('System', self.system_diagnostics)
        self.updater.add('Battery', self.battery_diagnostics)
        self.updater.add('User Voltage Supplies', self.voltage_diagnostics)
        self.updater.add('Current Consumption', self.current_diagnostics)

        # Add Frequency status
        self.imu_freq = HeaderlessTopicDiagnostic('imu/data_raw',
                                                  self.updater,
                                                  FrequencyStatusParam(self.imu_freq_bounds))

        self.mag_freq = HeaderlessTopicDiagnostic('imu/mag',
                                                  self.updater,
                                                  FrequencyStatusParam(self.imu_freq_bounds))

        self.feedback_freq = HeaderlessTopicDiagnostic('feedback',
                                                    self.updater,
                                                    FrequencyStatusParam(self.feedback_freq_bounds))

        self.navsat_freq = HeaderlessTopicDiagnostic('navsat/nmea_sentence',
                                                    self.updater,
                                                    FrequencyStatusParam(self.navsat_freq_bounds))

        self.updater.force_update()

    def system_diagnostics(self, stat):
        if self.stop_power_status:
            if self.external_stop_present:
                stat.add('External stop', 'Present, asserting stop')
            else:
                stat.add('External stop', 'Present, not asserting stop')
        else:
            stat.add('External stop', 'Absent')

        if self.external_stop_present:
            stat.summary(DiagnosticStatus.ERROR, 'External stop device asserting motor stop.')
        else:
            stat.summary(DiagnosticStatus.OK, 'System OK.') 
        return stat

    def battery_diagnostics(self, stat):
        vbat = self.measured_voltages[Power.JACKAL_MEASURED_BATTERY]

        if vbat > 30.0:
            stat.summary(DiagnosticStatus.ERROR, 'Battery overvoltage.')
        elif vbat < 1.0:
            stat.summary(DiagnosticStatus.ERROR, 'Battery voltage not detected, check BATT fuse.')
        elif vbat < 20.0:
            stat.summary(DiagnosticStatus.ERROR, 'Battery critically under voltage.')
        elif vbat < 24.0:
            stat.summary(DiagnosticStatus.WARN, 'Battery low voltage.')
        else:
            stat.summary(DiagnosticStatus.OK, 'Battery OK')

        stat.add('Battery Voltage (V)', '%.2f' % vbat)
        return stat

    def voltage_diagnostics(self, stat):
        v5 = self.measured_voltages[Power.JACKAL_MEASURED_5V]
        v12 = self.measured_voltages[Power.JACKAL_MEASURED_12V]

        if v12 > 12.5 or v5 > 5.5:
            stat.summary(DiagnosticStatus.ERROR, 'User supply overvoltage. Accessories may be damaged.')
        elif v12 < 1.0 or v5 < 1.0:
            stat.summary(DiagnosticStatus.ERROR, 'User supplies absent. Check tray fuses.')
        elif v12 < 11.0 or v5 < 4.0:
            stat.summary(DiagnosticStatus.WARN, 'Voltage supplies undervoltage. Check loading levels.')
        else:
            stat.summary(DiagnosticStatus.OK, 'User supplies OK.')

        stat.add('12V Supply (V)', '%.2f' % v12)
        stat.add('5V Supply (V)', '%.2f' % v5)
        return stat

    def current_diagnostics(self, stat):
        current_computer = self.measured_currents[Power.JACKAL_COMPUTER_CURRENT]
        current_drive = self.measured_currents[Power.JACKAL_DRIVE_CURRENT]
        current_user = self.measured_currents[Power.JACKAL_USER_CURRENT]
        current_total = self.measured_currents[Power.JACKAL_TOTAL_CURRENT]

        if current_total > 32.0:
            stat.summary(DiagnosticStatus.ERROR, 'Current draw critical.')
        elif current_total > 20.0:
            stat.summary(DiagnosticStatus.WARN, 'Current draw warning.')
        elif current_total > 10.0:
            stat.summary(DiagnosticStatus.OK, 'Current draw requires monitoring.')
        else:
            stat.summary(DiagnosticStatus.OK, 'User supplies OK.')

        stat.add('Drive Current (A)', '%.2f' % current_drive)
        stat.add('User Current (A)', '%.2f' % current_user)
        stat.add('Computer Current (A)', '%.2f' % current_computer)
        stat.add('Total Current (A)', '%.2f' % current_total)
        return stat

    def power_callback(self, msg: Power):
        self.measured_currents = msg.measured_currents
        self.measured_voltages = msg.measured_voltages

    def imu_callback(self, msg):
        self.imu_freq.tick()

    def mag_callback(self, msg):
        self.mag_freq.tick()

    def feedback_callback(self, msg):
        self.feedback_freq.tick()

    def navsat_callback(self, msg):
        self.navsat_freq.tick()

    def status_callback(self, msg: Status):
        self.updater.setHardwareID(msg.hardware_id)
        self.updater.update()

    def stop_status_callback(self, msg: StopStatus):
        self.stop_power_status = msg.stop_power_status
        self.external_stop_present = msg.external_stop_present

    


def main(args=None):
    rclpy.init(args=args)

    node = JackalDiagnosticsUpdater()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
