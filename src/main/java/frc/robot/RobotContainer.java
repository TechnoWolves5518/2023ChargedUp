/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.DriveCommands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  DriveSubsystem driveSubsystem = new DriveSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * print("".join([i for i in "Hello World"]))
   */

  private void configureButtonBindings() {
    Joystick controller = new Joystick(JoystickConstants.controllerPortID);

    JoystickButton rightBumperButton = new JoystickButton(controller, JoystickConstants.rightBumperButtonID);
    DoubleSupplier leftJoystickX = () -> deadband(controller.getRawAxis(JoystickConstants.leftJoystickXAxis), 0.1);
    DoubleSupplier leftJoystickY = () -> deadband(controller.getRawAxis(JoystickConstants.leftJoystickYAxis), 0.1);
    DoubleSupplier rightJoystickX = () -> deadband(controller.getRawAxis(JoystickConstants.rightJoystickXAxis), 0.1);
    BooleanSupplier rightBumper = () -> rightBumperButton.getAsBoolean();

    driveSubsystem
        .setDefaultCommand(new DriveCommand(driveSubsystem, leftJoystickX, leftJoystickY, rightJoystickX, rightBumper));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

}
