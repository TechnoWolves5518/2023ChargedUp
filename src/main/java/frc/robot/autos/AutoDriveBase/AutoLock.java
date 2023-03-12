package frc.robot.autos.AutoDriveBase;

import frc.robot.Constants;
import frc.robot.Constants.SwerveDrive;
import frc.robot.subsystems.Swerve;



import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoLock extends CommandBase {    
    private Swerve s_Swerve;    
    public AutoLock(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = 0;
        double strafeVal = 0;
        double rotationVal = 0.05;
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveDrive.maxSpeed), 
            rotationVal * Constants.SwerveDrive.maxAngularVelocity, 
            true, 
            true);
        
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}