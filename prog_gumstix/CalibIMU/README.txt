To calibrate acceloremeter:
1) Place robot in horizontal position.
2) Press Record switch during few seconds.
3) Place robot in another position and press Record switch few seconds.
4) Repeat step 5) as much as you want.
5) Stop model.
6) Execute calc_bias_fact.m to calculate bias b and scale factor T of the acceloremeter.

calc_bias_fact.m parameters':
- N : number of observation point for each orientation of the robot.