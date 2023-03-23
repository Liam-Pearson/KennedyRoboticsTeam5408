# KennedyRoboticsTeam5408

Directions:
[N] - Front of Robot
[E] - Right of Robot
[S] - Rear of Robot (Battery is in the rear)
[W] - Left of Robot

Axis Mapping:
[A1] - X-Axis
[A2] - Y-Axis
[A3] - Z-Axis
[A4] - SpeedAxis

Button Mapping:
[B1] (front trigger) - Pulls arm upwards by rotating the pulley North
[B2] - Lowers the arm by rotating the pulley South
[B3] - Closes the claw
[B4] - Opens the claw
[B5] - Extends the telescoping arm
[B6] - Retracts (reduces tension on wire, allowing it to fall back in place) the telescoping arm.
[B11] - Emergency Stop!
[B12] - Exit Emergency Stop!

Operating Procedures:
[1] - Open FRC Driver Station
[2] - Connect battery
[3] - Turn on robot
[4] - Connect to robot wifi (5408 bing bong is the network name; it may take a minute for the radio to boot up and be available to connect to)
[5] - Verify that Communications, Robot Code, and Joystick are green. If Joystick is not green, go into the USB Order Tab (wire square on left side of the screen) and ReScan for USB devices.
[6] - Set Speed slider to 0 (down position).
[7] - Verify joystick input in FRC PC Dashboard.
        - If the joystick isn't appaearing, rescan for it in the USB tab in FRC Driver Station.
[8] - Enable the robot.
[9] - Verify that the robot can move and that the robot moves in the correct directions based on the inputs of the joystick.
        - Ensure that the speed slider functions.
            - Drive the robot forwards, then backwards, then left, then right with the slider set to 0.
            - Drive the robot forwards, then backwards, then left, then right with the slider set to the middle position.
            - Drive the robot forwards, then backwards, then left, then right with the slider set to the full forwards position.
[10] - Verify that the arm-lifting pulley system works.
        - Ensure that the L-bracket that directs the wire is attached tightly and that the bolt is not in contact with the spool.
        - Ensure that the pulley does not smoke (if it does that is bad).
        - Ensure the wire is not caught in the pulleys.
        - Lift the arm up and then down.
[11] - Verify that the claw works.
        - Verify that the compressor turns on.
        - Verify that the solenoid lights up (yellow).
        - Verify that the piston moves up and down.
[12] - Verify that the arm extending system works.
        - Ensure that the brackets directing the wire are tightly attached.
        - Ensure that the wire is not caught in the pulleys.
        - Ensure that the arm can extend out at a maximum of 45 degrees (highest operating evelope [HOE]).
        - Ensure that the arm can retract inwards at a maximum of 45 degrees (highest operating envelope [HOE]).

Troubleshooting Procedures:
- If the robot does not work. Panic, then follow the following:
[1] - Disable, then re-enable the robot.
[2] - If step #2 does not work, re-deploy the code to the robot and repeat step #1.
[3] - If step #2 does not work, restart all software being used, or restart the computer. Repeat all following steps.
[4] - If all following steps do not work, turn the robot on and off, then repeat all following steps.
[5] - If you are not recieving printouts, check the connection of the computer to the radio, then check the connection of the radio tothe robot.
[6] - If all previous steps do not work, check code for any errors. Common errors could be incorrect boolean states, missing or incorrect semicolons, or missing or incorrect closing brackets.
[7] - If all previous steps do not work, check wiring and connections. If motors are not functioning, check the motor controllers and connections. If connected properly, test if the motor is burnt out. If you have sensors that are not working, or are giving inadmissable readings, you're screwed, because I haven't fixed that yet either. If pistons are not working, check that all tubing is correct and that you are at the correct operating pressure. Operating pressure can be read from the dials which you should have in your pneumatic system.
[8] - If robot is still not working, I'm not sure how to help. I'd advice checking the WPILib documentation and FRC documentation.