package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* Motor */
PWMVenom arm = new PWMVenom(0); // TODO: Change port

/* Commands */

// TODO: Add angles

special.extend()
    .whileTrue(new extendArm(arm.set()));

special.retract()
    .whileTrue(new retractArm(arm.set()));
