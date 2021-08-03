import fcntl
import itertools
import os

from io import FileIO
from time import sleep

from evdev import InputDevice
from pyudev import Context, Monitor


class DS4Report(object):
    __slots__ = ["left_analog_x",
                 "left_analog_y",
                 "right_analog_x",
                 "right_analog_y",
                 "l2_analog",
                 "r2_analog",
                 "dpad_up",
                 "dpad_down",
                 "dpad_left",
                 "dpad_right",
                 "button_cross",
                 "button_circle",
                 "button_square",
                 "button_triangle",
                 "button_l1",
                 "button_l2",
                 "button_l3",
                 "button_r1",
                 "button_r2",
                 "button_r3",
                 "button_share",
                 "button_options",
                 "button_trackpad",
                 "button_ps",
                 "motion_y",
                 "motion_x",
                 "motion_z",
                 "orientation_roll",
                 "orientation_yaw",
                 "orientation_pitch",
                 "trackpad_touch0_id",
                 "trackpad_touch0_active",
                 "trackpad_touch0_x",
                 "trackpad_touch0_y",
                 "trackpad_touch1_id",
                 "trackpad_touch1_active",
                 "trackpad_touch1_x",
                 "trackpad_touch1_y",
                 "timestamp",
                 "battery",
                 "plug_usb",
                 "plug_audio",
                 "plug_mic"]

    def __init__(self, *args, **kwargs):
        for i, value in enumerate(args):
            setattr(self, self.__slots__[i], value)


class DS4Device(object):
    """A DS4 controller object.
    Used to control the device functions and reading HID reports.
    """

    def __init__(self, device_name, device_addr, type):
        self.device_name = device_name
        self.device_addr = device_addr
        self.type = type

        self._led = (0, 0, 0)
        self._led_flash = (0, 0)
        self._led_flashing = False

        self.set_operational()

    def _control(self, **kwargs):
        self.control(led_red=self._led[0], led_green=self._led[1],
                     led_blue=self._led[2], flash_led1=self._led_flash[0],
                     flash_led2=self._led_flash[1], **kwargs)

    def rumble(self, small=0, big=0):
        """Sets the intensity of the rumble motors. Valid range is 0-255."""
        self._control(small_rumble=small, big_rumble=big)

    def set_led(self, red=0, green=0, blue=0):
        """Sets the LED color. Values are RGB between 0-255."""
        self._led = (red, green, blue)
        self._control()

    def start_led_flash(self, on, off):
        """Starts flashing the LED."""
        if not self._led_flashing:
            self._led_flash = (on, off)
            self._led_flashing = True
            self._control()

    def stop_led_flash(self):
        """Stops flashing the LED."""
        if self._led_flashing:
            self._led_flash = (0, 0)
            self._led_flashing = False
            # Call twice, once to stop flashing...
            self._control()
            # ...and once more to make sure the LED is on.
            self._control()

    def control(self, big_rumble=0, small_rumble=0,
                led_red=0, led_green=0, led_blue=0,
                flash_led1=0, flash_led2=0):
        if self.type == "bluetooth":
            pkt = bytearray(77)
            pkt[0] = 128
            pkt[2] = 255
            offset = 2
            report_id = 0x11

        elif self.type == "usb":
            pkt = bytearray(31)
            pkt[0] = 255
            offset = 0
            report_id = 0x05

        # Rumble
        pkt[offset+3] = min(small_rumble, 255)
        pkt[offset+4] = min(big_rumble, 255)

        # LED (red, green, blue)
        pkt[offset+5] = min(led_red, 255)
        pkt[offset+6] = min(led_green, 255)
        pkt[offset+7] = min(led_blue, 255)

        # Time to flash bright (255 = 2.5 seconds)
        pkt[offset+8] = min(flash_led1, 255)

        # Time to flash dark (255 = 2.5 seconds)
        pkt[offset+9] = min(flash_led2, 255)

        self.write_report(report_id, pkt)

    def parse_report(self, buf):
        """Parse a buffer containing a HID report."""
        dpad = buf[5] % 16

        return DS4Report(
            # Left analog stick
            buf[1], buf[2],

            # Right analog stick
            buf[3], buf[4],

            # L2 and R2 analog
            buf[8], buf[9],

            # DPad up, down, left, right
            (dpad in (0, 1, 7)), (dpad in (3, 4, 5)),
            (dpad in (5, 6, 7)), (dpad in (1, 2, 3)),

            # Buttons cross, circle, square, triangle
            (buf[5] & 32) != 0, (buf[5] & 64) != 0,
            (buf[5] & 16) != 0, (buf[5] & 128) != 0,

            # L1, L2 and L3 buttons
            (buf[6] & 1) != 0, (buf[6] & 4) != 0, (buf[6] & 64) != 0,

            # R1, R2,and R3 buttons
            (buf[6] & 2) != 0, (buf[6] & 8) != 0, (buf[6] & 128) != 0,

            # Share and option buttons
            (buf[6] & 16) != 0, (buf[6] & 32) != 0,

            # Trackpad and PS buttons
            (buf[7] & 2) != 0, (buf[7] & 1) != 0,

            # Acceleration
            S16LE.unpack_from(buf, 13)[0],
            S16LE.unpack_from(buf, 15)[0],
            S16LE.unpack_from(buf, 17)[0],

            # Orientation
            -(S16LE.unpack_from(buf, 19)[0]),
            S16LE.unpack_from(buf, 21)[0],
            S16LE.unpack_from(buf, 23)[0],

            # Trackpad touch 1: id, active, x, y
            buf[35] & 0x7f, (buf[35] >> 7) == 0,
            ((buf[37] & 0x0f) << 8) | buf[36],
            buf[38] << 4 | ((buf[37] & 0xf0) >> 4),

            # Trackpad touch 2: id, active, x, y
            buf[39] & 0x7f, (buf[39] >> 7) == 0,
            ((buf[41] & 0x0f) << 8) | buf[40],
            buf[42] << 4 | ((buf[41] & 0xf0) >> 4),

            # Timestamp and battery
            buf[7] >> 2,
            buf[30] % 16,

            # External inputs (usb, audio, mic)
            (buf[30] & 16) != 0, (buf[30] & 32) != 0,
            (buf[30] & 64) != 0
        )

    def read_report(self):
        """Read and parse a HID report."""
        pass

    def write_report(self, report_id, data):
        """Writes a HID report to the control channel."""
        pass

    def set_operational(self):
        """Tells the DS4 controller we want full HID reports."""
        pass

    def close(self):
        """Disconnects from the device."""
        pass

    @property
    def name(self):
        if self.type == "bluetooth":
            type_name = "Bluetooth"
        elif self.type == "usb":
            type_name = "USB"

        return "{0} Controller ({1})".format(type_name, self.device_name)


class FakeDS4Device(DS4Device):
    __type__ = 'usb'
    report_size = 64
    valid_report_id = 0x01

    def set_operational(self):
        pass

    def __init__(self): #, name, addr, type, hidraw_device, event_device):
        try:
            self.report_fd = os.open('/home/rover/fakeds4reportfd', os.O_RDWR | os.O_NONBLOCK)
            self.fd = FileIO(self.report_fd, "rb+", closefd=False)


        except (OSError, IOError) as err:
            print("Have encountered an error while handling file opening stuff.")
            print(err)
            raise err
            #raise DeviceError(err)

        self.buf = bytearray(self.report_size)

        super(FakeDS4Device, self).__init__('fake_ds4_device', '0.0.0.0.0.0', 'usb')

    def read_report(self):
        ret = DS4Report()
        print("Returning empty ds4 report.")
        return ret


	try:
            ret = self.fd.readinto(self.buf)
        except IOError:
            return

        # Disconnection
        if ret == 0:
            return

        # Invalid report size or id, just ignore it
        if ret < self.report_size or self.buf[0] != self.valid_report_id:
            print("Invalid report read")
            return False

        if self.type == "bluetooth":
            # Cut off bluetooth data
            buf = zero_copy_slice(self.buf, 2)
        else:
            buf = self.buf

        return self.parse_report(buf)

    def read_feature_report(self, report_id, size):
        op = HIDIOCGFEATURE(size + 1)
        buf = bytearray(size + 1)
        buf[0] = report_id

        return fcntl.ioctl(self.fd, op, bytes(buf))

    def write_report(self, report_id, data):
        hid = bytearray((report_id,))
        self.fd.write(hid + data)

    def close(self):
        try:
            # Reset LED to original hidraw pairing colour.
            self.set_led(0, 0, 1)

            self.fd.close()
        except IOError:
            pass


class HidrawBackend:
    __name__ = "hidraw"

    def setup(self):
        pass

    @property
    def devices(self):
        """Wait for new DS4 devices to appear."""
        yield FakeDS4Device()
