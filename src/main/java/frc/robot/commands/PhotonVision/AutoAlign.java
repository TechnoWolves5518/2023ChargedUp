package frc.robot.commands.PhotonVision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.RotationConstants;


public class AutoAlign extends CommandBase {
  private final Swerve driveSubsystem;
  private final Vision vision;
  private boolean isAligned;
  private PIDController rotationPID;
  boolean interrupted = false;

  private Supplier<Double> xspeedSupplier;
  private Supplier<Double> yspeedSupplier;


  
  PIDController controller = CameraConstants.driveController;

  public AutoAlign(
    Swerve driveSubsystem, 
    Vision vision,
    Supplier<Double> i, 
    Supplier<Double> j) {
    this.driveSubsystem = driveSubsystem;
    this.vision = vision;
    this.xspeedSupplier = i;
    this.yspeedSupplier = j;

    rotationPID = new PIDController(RotationConstants.kP, RotationConstants.kI, RotationConstants.kD);
    rotationPID.enableContinuousInput(RotationConstants.kMinimumAngle, RotationConstants.kMaximumAngle);

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAligned = false;
    var robotPose = driveSubsystem.getPose();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed;
    double x_speed;
    double xSpeed = xspeedSupplier.get();
    double y_speed; 
    double ySpeed = yspeedSupplier.get();
    var robotPose2d = driveSubsystem.getPose();
    System.out.println("running");
    if (vision.getTargetAngle() < 2) {
      interrupted = true;
    }

    if (vision.hasTargets == true) {
      double currentHeading = driveSubsystem.getvisionheading();
      double rotation = rotationPID.calculate(currentHeading,0);

      x_speed = controller.calculate(vision.getTargetAngle(), 0);
      y_speed = controller.calculate(vision.getTargetAngle(), 0);

      driveSubsystem.drive(new Translation2d(x_speed,ySpeed), rotation,true, true);
    } else {
      double currentHeading = driveSubsystem.getvisionheading();
      double rotation = rotationPID.calculate(currentHeading,0);
      driveSubsystem.drive(new Translation2d(xSpeed,ySpeed),rotation,true,true);
    }
  }
  @Override
  public void end(boolean interrupted) {  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}