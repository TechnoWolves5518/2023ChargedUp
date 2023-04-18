package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.AutoCommands.AutoHighScore;
import frc.robot.autos.AutoDriveBase.AutoRotate;
import frc.robot.commands.armRotator.GoToDefaultState;
import frc.robot.subsystems.ArmExtender;
import frc.robot.subsystems.ArmSpinner;
import frc.robot.subsystems.BrakeArm;
import frc.robot.subsystems.HandGripper;
import frc.robot.subsystems.HandSpinner;
import frc.robot.subsystems.Swerve;



public class AutoSelector {
    
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  
  public AutoSelector(Swerve drivebase, HandGripper h_Gripper, ArmSpinner a_Spinner, BrakeArm b_Arm, HandSpinner h_Spinner, ArmExtender a_Extender) {
    chooser.setDefaultOption("Showcase Spin", new SequentialCommandGroup(
      new AutoRotate(drivebase)
    ));

    chooser.addOption("Score and done", new SequentialCommandGroup(
      new AutoHighScore(a_Spinner, b_Arm, h_Gripper, h_Spinner, a_Extender),
      new GoToDefaultState(a_Spinner, b_Arm, a_Extender, h_Gripper)
    ));
    
      
    SmartDashboard.putData(chooser);
  }


  public Command getSelected() {
    return chooser.getSelected();
  }
}