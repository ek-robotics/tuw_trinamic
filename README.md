# TUW TRINAMIC

## `tuw_trinamic`
Metapackage for `tuw trinamic`

## `tuw_trinamic_iwos_revolute_controller`
Package to control brushless DC motors with [Trinamic TMCM-1640][TrinamicTMCM-1640] with [JointIWS][JointIWS] messages from the [`tuw_msgs` package][tuw_msgs] where `cmd_velocity` (unit: m/s) vales are applied to the motor.
To move the wheel attached to the motor with the correct angular velocity the wheel diameter as well as other motor specific values are stored in the `annoy.yaml` file.

To communicate with the Trinamic TMCM-1640 board the [PyTrinamic repository][PyTrinamic] (branch: master) is utilized.

To launch the iwos velocity controller node run:
```bash
roslaunch tuw_trinamic_iwos_velocity_controller iwos_velocity_controller.launch
```

[TrinamicTMCM-1640]: https://www.trinamic.com/products/modules/details/tmcm-1640/ 
[JointIWS]: https://github.com/tuw-robotics/tuw_msgs/blob/master/tuw_nav_msgs/msg/JointsIWS.msg
[tuw_msgs]: https://github.com/tuw-robotics/tuw_msgs
[PyTrinamic]: https://github.com/trinamic/PyTrinamic