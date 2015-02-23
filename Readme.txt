BD rate: 19200
Output: X position; Y position; Rotation Speed; Length of Linear Actuator (Params in MotorParam.h)

Commands:
gc: calibration

[x/y][a(bsolute)/r(elative)][value]+end mark: move in X/Y direction
[r][o(penloop)/c(losekoop)][speed value, 0..400]+end mark: rotation
[l][p(osition control)/h(it)][20..885](no value needed for hit mode)+end mark: linear actuator

End mark: any non number symbol

*If the motor sleeps, reset Mega
