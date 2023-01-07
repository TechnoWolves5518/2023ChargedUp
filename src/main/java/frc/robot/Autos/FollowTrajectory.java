package frc.robot.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.RobotMap;
import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.platform.can.AutocacheState;

public class FollowTrajectory extends CommandBase {
  private final Swerve swerve;
  private Trajectory trajectory;
  private boolean toReset;

  public FollowTrajectory(Swerve swerve, String trajectoryFilePath, boolean toReset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.toReset = toReset;
    addRequirements(swerve);
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFilePath);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("unable to open trajectory: " + trajectoryFilePath, 
        e.getStackTrace());
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (toReset) {
      swerve.resetOdemetry(trajectory.getInitialPose());
    }

    final ProfiledPIDController thetaController =
    new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    new SwerveControllerCommand(
      trajectory, 
      swerve::getPose, 
      RobotMap.swerveKinematics, 
      
      new PIDController(AutoConstants.kPXController, 0 , 0), 
      new PIDController(AutoConstants.kPYController, 0, 0), 
      thetaController, 
      swerve::setModuleStates, 
      swerve);
      System.out.println("autonomous is running");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("autonomous complete");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
