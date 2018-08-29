# A ROS node converting rosflight to mav_msgs

Note that `rosflight` is in the NED frame, while `mav_msgs` is in NWU frame. The conversionis being made interally for this in IMU and commands, which are presented in the NWU frame.

## Contributors

* [Emil Fresk](https://www.github.com/korken89)

---

## License

Licensed under the Boost Software License 1.0, see LICENSE file for details.

---

## Functionality

Converts the main command and sensor topics to mav_msgs format. Still WIP.
