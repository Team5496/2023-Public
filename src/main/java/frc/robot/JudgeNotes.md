# Started with small prototype arm
    - Never worked with SparkMax motor controllers and needed to learn how to use/tune them.
    - Spent a long time trying to figure out how to tune the controller correctly
    - Time well spent, made understanding the PIDs for the Sparks a lot easier and reduced time spent tuning the actual robot later
# Tuning the actual robot arm
    - Started off strong, tuning the elevator was easy with the notes from tuning the prototype
    - Tuning the arm seemed simple enough and we got the intake position tuned relatively quickly
    - Quickly realized the arm we designed did not have a linear torque which made tuning different positions a struggle because as soon as the torque began getting smaller, the kP was too high for the new torque the motors were trying to move against, which would send the arm into wild oscillations
    - To solve the variable torque issue, we switched to chain belts on the robot arm, which prevented skipping and increased consistency, and tuned four different PID loops for each of the torque zones we had to move through in hopes we could switch the correct values in and out as needed to keep complete control of the motors
        - Tuning those four loops didn't take a super long time, but we were still having issues with consistency and the arm oscillating wildly if we missed a set point even slightly
    - We couldn't tune out the wild oscillations so added another 7:1 gearbox on each arm, bringing the total from 27:1 to 189:1 as well as switching from using kI in our tuning to kD
    - Spent another while retuning everything for new gear ratio, discovering our values decreased rather than increasing like we expected, eventually getting it to work and moving on to trying to up the speed of transitions
# Changing the design
    - After our first comp we discovered that the design for the arm and intake was inefficient and floppy
    - Changed our elevator + 4-bar arm to two elevators with a drop down arm
        - Also changed to using a Falcon for the arm control rather than the SparkMax and Rev motors in order to be able to use motion magic with trapezoidal motion profiling instead of a position closed loop

