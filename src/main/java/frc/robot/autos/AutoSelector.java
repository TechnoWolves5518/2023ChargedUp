package frc.robot.autos;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveDrive;
import frc.robot.autos.AutoCommands.AutoGroundPickup;
import frc.robot.autos.AutoCommands.AutoHighScore;
import frc.robot.autos.AutoCommands.AutoOpen;
import frc.robot.autos.AutoCommands.DelayAction;
import frc.robot.autos.AutoDriveBase.AutoBalance;
import frc.robot.autos.AutoDriveBase.AutoDriveBack;
import frc.robot.autos.AutoDriveBase.AutoDriveForward;
import frc.robot.autos.AutoDriveBase.AutoLock;
import frc.robot.commands.ArmExtender.ExtendArm;
import frc.robot.commands.armRotator.GoToDefaultState;
import frc.robot.commands.armRotator.GoToPassiveStage;
import frc.robot.subsystems.ArmExtender;
import frc.robot.subsystems.ArmSpinner;
import frc.robot.subsystems.BrakeArm;
import frc.robot.subsystems.HandGripper;
import frc.robot.subsystems.HandSpinner;
import frc.robot.subsystems.Swerve;



public class AutoSelector {
    
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  //define autonomous routines
  PathPlannerTrajectory northAutoBalance = PathPlanner.loadPath("NorthAutoBalance", new PathConstraints(4, 2));
  PathPlannerTrajectory testPath = PathPlanner.loadPath("TestPath", new PathConstraints(4, 2));
  PathPlannerTrajectory southAutoBalance = PathPlanner.loadPath("SouthAutoBalance", new PathConstraints(4, 2));
  PathPlannerTrajectory northAutoBail = PathPlanner.loadPath("NorthAutoBail", new PathConstraints(4, 2));
  PathPlannerTrajectory southAutoBail = PathPlanner.loadPath("SouthAutoBail", new PathConstraints(4, 2));
  
  public AutoSelector(Swerve drivebase, HandGripper h_Gripper, ArmSpinner a_Spinner, BrakeArm b_Arm, HandSpinner h_Spinner, ArmExtender a_Extender) {
    //I'm not gonna try and figure out something better for the time being
    chooser.addOption("NorthAutoLevel", new SequentialCommandGroup(
    new AutoHighScore(a_Spinner, b_Arm, h_Gripper, h_Spinner, a_Extender),
    new GoToDefaultState(a_Spinner, b_Arm, a_Extender, h_Gripper),
    new InstantCommand(() -> {
        // Reset odometry for the fite[]\78+78/rst path you run during auto
        drivebase.resetOdometry(northAutoBalance.getInitialHolonomicPose());
      }), 
      new PPSwerveControllerCommand(
        northAutoBalance,
         drivebase::getPose, // Pose supplier
       SwerveDrive.swerveKinematics, // SwerveDriveKinematics
         new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
         new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
         new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
         drivebase::setModuleStates, // Module states consumer
         true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
         drivebase // Requires this drive subsystem
     ), new AutoDriveBack(drivebase), new AutoBalance(drivebase), new AutoLock(drivebase)
      ));

      chooser.addOption("SouthAutoLevel", new SequentialCommandGroup(
        new AutoHighScore(a_Spinner, b_Arm, h_Gripper, h_Spinner, a_Extender),
        new GoToDefaultState(a_Spinner, b_Arm, a_Extender, h_Gripper),
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(southAutoBalance.getInitialHolonomicPose());
        }), 
        new PPSwerveControllerCommand(
          southAutoBalance,
           drivebase::getPose, // Pose supplier
         SwerveDrive.swerveKinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           drivebase::setModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           drivebase // Requires this drive subsystem
       ), new AutoDriveBack(drivebase), new AutoBalance(drivebase), new AutoLock(drivebase)
        ));
      
      chooser.addOption("NorthAutoBail", new SequentialCommandGroup(
        new AutoHighScore(a_Spinner, b_Arm, h_Gripper, h_Spinner, a_Extender),
        new GoToDefaultState(a_Spinner, b_Arm, a_Extender, h_Gripper),
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(northAutoBail.getInitialHolonomicPose());
        }), 
        new PPSwerveControllerCommand(
          northAutoBail,
           drivebase::getPose, // Pose supplier
         SwerveDrive.swerveKinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           drivebase::setModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           drivebase // Requires this drive subsystem
       )
      ));

      chooser.addOption("SouthAutoBail", new SequentialCommandGroup(
        new AutoHighScore(a_Spinner, b_Arm, h_Gripper, h_Spinner, a_Extender),
        new GoToDefaultState(a_Spinner, b_Arm, a_Extender, h_Gripper),
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(southAutoBail.getInitialHolonomicPose());
        }), 
        new PPSwerveControllerCommand(
          southAutoBail,
           drivebase::getPose, // Pose supplier
         SwerveDrive.swerveKinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           drivebase::setModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           drivebase // Requires this drive subsystem
       )
      ));

      chooser.setDefaultOption("Contingency", new SequentialCommandGroup(
      new AutoHighScore(a_Spinner, b_Arm, h_Gripper, h_Spinner, a_Extender),
      new GoToDefaultState(a_Spinner, b_Arm, a_Extender, h_Gripper),
      new AutoDriveForward(drivebase), 
      new AutoBalance(drivebase), 
      new AutoLock(drivebase)));

      chooser.addOption("TestAuto", new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(testPath.getInitialHolonomicPose());
        }),
        new PPSwerveControllerCommand(
          testPath,
           drivebase::getPose, // Pose supplier
         SwerveDrive.swerveKinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           drivebase::setModuleStates, // Module states consumer
           false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           drivebase // Requires this drive subsystem
       )
         /*new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(southAutoBail.getInitialHolonomicPose());
        }), 
        new PPSwerveControllerCommand(
          testPath,+
           drivebase::getPose, // Pose supplier
         SwerveDrive.swerveKinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           drivebase::setModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           drivebase // Requires this drive subsystem
       )
      )*/
      ));
      
    SmartDashboard.putData(chooser);
  }


  public Command getSelected() {
    return chooser.getSelected();
  }
}