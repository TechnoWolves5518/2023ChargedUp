// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.armSpinner;
import frc.robot.RobotContainer;

public class rotateArm extends CommandBase {
  /** Creates a new moveArm. */

  public rotateArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    
    public static double armSpinSpeed = new armSpinSpeed(double);

    XboxController specialSpinner = RobotContainer.special;
    final armSpinner s_Spin = new armSpinner(); 
    addRequirements(s_Spin);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean forwards = specialSpinner.getAButton();
    boolean backwards = specialSpinner.getBButton();

    if (forwards && backwards == false){
      speed = 0;
    } else if(forwards == true){
      speedMod = armSpinSpeed;
    } else if(backwards == true){
      speedMod = -1 * armSpinSpeed;

    }

    armPwmVenom.set(armSpinSpeed);







  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; }
}