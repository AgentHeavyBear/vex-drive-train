# vex-drive-train
VEX Drive train class for PROS C++. Meant to make it easier to program the driving functions of the robot, especially for autonomous. It abstracts multiple drive motors under a single class, the class contains most or all of the movement and telemetry functions available to each individual motor, and applies them to the whole drive base, while also leaving the individual motors available to call normally.