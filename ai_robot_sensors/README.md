# ai_robot_sensors

This package manages all sensors used with the ai_robot project.

## udev rules

**udev** is a device manager for Linux that dynamically manages hardware devices. Linux enumerates
hardware devices attached to the computer and creates a device file for each of them. However, the
name of a device file depends on the order in which the corresponding device gets detected, and the
order may alter upon each system boot. With udev, we are able to create a rule for a specific
hardware device to fix the path to its device file.

Refer to <https://www.clearpathrobotics.com/assets/guides/noetic/ros/index.html> for more about how
to set up udev rules.

The `rules` directory contains pre-defined udev rule files. Pick some of them as needed and install
them to `/etc/udev/rules.d/`. Note that some pre-defined rules might not fit, and you may have
to modify them or even create new rules to match your current hardware configuration.

To find out attributes of a device like vendor id, product id and serial number, run:

```bash
udevadm info -a --name=<devpath>
```

or more accurately

```bash
udevadm info -a -p $(udevadm info -q path -n <devpath>)
```
