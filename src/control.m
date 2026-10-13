% Control: joint angles -> servo commands -> arm.
% - Servo fit s = a*theta + b, per joint, using this arm's numbers
% - Send servo commands to the arm (transport depends on the PC vs microcontroller decision)
% - Break long moves into small steps and wait after each, since servos report no position
