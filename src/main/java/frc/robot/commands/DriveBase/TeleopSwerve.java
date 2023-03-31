package frc.robot.commands.DriveBase;

import frc.robot.Constants;
import frc.robot.Constants.SwerveDrive;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private boolean brakeCheck;
    private final XboxController driver = new XboxController(0);
    private boolean driveSlowCheck;
    private boolean rotateSlowCheck;
    private double driveSpeed;
    private double rotateSpeed;
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble() * driveSpeed, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble() * driveSpeed, Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble() * rotateSpeed, Constants.stickDeadband);
        brakeCheck = driver.getXButton();
        driveSlowCheck = driver.getRightBumper();
        rotateSlowCheck = driver.getLeftBumper();
        //System.out.println(brakeCheck);
        /* Drive */
        if (brakeCheck == true) {
            translationVal = 0;
            strafeVal = 0;
            rotationVal = 0.05;
        }
        if (driveSlowCheck == true) {
            driveSpeed = SwerveDrive.slowMod;
        } else {
            driveSpeed = SwerveDrive.speedMod;
        }
        
        if (rotateSlowCheck == true) {
            rotateSpeed = SwerveDrive.slowMod;
        } else {
            rotateSpeed = SwerveDrive.speedMod;
        }
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveDrive.maxSpeed), 
            rotationVal * Constants.SwerveDrive.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true);
        
    }
}