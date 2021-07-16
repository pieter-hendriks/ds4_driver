from .controller import Controller
from . import path

import rospy
from ds4_driver.msg import Status


class ControllerRos(Controller):
    def __init__(self):
        super(ControllerRos, self).__init__()
        assert not self.use_standard_msgs
        self.startTime = None
        # TODO: Add non-default/configurable path.
        self.path = path.getPath(3, 0.3)
        self.use_standard_msgs = rospy.get_param('~use_standard_msgs', False)
        self.deadzone = rospy.get_param('~deadzone', 0.1)
        self.frame_id = rospy.get_param('~frame_id', 'ds4')
        self.imu_frame_id = rospy.get_param('~imu_frame_id', 'ds4_imu')
        
        # Only publish Joy messages on change
        self._autorepeat_rate = rospy.get_param('~autorepeat_rate', 0)
        self._prev_joy = None
        self.pub_status = rospy.Publisher('status', Status, queue_size=1)

    def getTime(self):
        if self.startTime is None:
            self.startTime = rospy.Time.now()
            return rospy.Time(0)
        return rospy.Time.now() - self.startTime

    def cb_report(self, report):
        """
        Callback method for ds4drv event loop
        :param report:
        :return:
        """
        assert not self.use_standard_msgs # Unsupported at this time; see original code if it needs to be implemented.
        status = Status()
        # Copy header values that original code set
        status.header.frame_id = self.frame_id
        status.header.stamp = rospy.Time.now()
        status.imu.header.frame_id = self.imu_frame_id
        # Per config/twist_2dof.yaml, these two values control the speed and rotation
        status.axis_left_y = self.path.getCurrentValue(self.getTime(), path.Path.SPEED)
        status.axis_right_x = self.path.getCurrentValue(self.getTime(), path.Path.ROTATION)
        # Set battery to max in case that would cause any issues
        # Think there's some low battery indication we'll avoid this way
        status.battery_percentage = 1.0
        
        self.pub_status.publish(status)
        return
        
    def cb_feedback(self, msg):
        """
        Callback method for ds4_driver/Feedback
        :param msg:
        :type msg: Feedback
        :return:
        """
        # We don't implement this method
        # There's no actual controller, so we can't use controller feedback
        return

    def cb_stop_rumble(self, event):
        # We don't implement this method
        # There's no actual controller, so we can't use controller feedback
        return

    def cb_joy_feedback(self, msg):
        """
        Callback method for sensor_msgs/JoyFeedbackArray
        The message contains the following feedback:
        LED0: red
        LED1: green
        LED2: blue
        RUMBLE0: rumble small
        RUMBLE1: rumble big
        :param msg:
        :type msg: JoyFeedbackArray
        :return:
        """
        # We don't implement this method
        # There's no actual controller, so we can't use controller feedback
        return
    
    def cb_joy_pub_timer(self, _):
        # We don't implement this method
        return

    @staticmethod
    def _report_to_status_(report_msg, deadzone=0.05):
        # We don't implement this method
        return

    @staticmethod
    def _normalize_axis_(val, deadzone=0.0):
        # We don't implement this method
        return

    @staticmethod
    def _status_to_joy_(status):
        # We don't implement this method
        return

    @staticmethod
    def _status_to_battery_(status):
        # We don't implement this method
        return

    @staticmethod
    def _status_to_imu_(status):
        # We don't implement this method
        return