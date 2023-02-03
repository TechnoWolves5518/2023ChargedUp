package frc.robot.autos.AutoCommands;

import frc.robot.Constants;
import frc.robot.Constants.SwerveDrive;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoLock extends CommandBase {    
    private Swerve s_Swerve;    
    //delete these "unused variables" if you want the code to break
    
    private BooleanSupplier robotCentricSup;
    private boolean brakeCheck;
    private final XboxController driver = new XboxController(0);
    public AutoLock(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
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