package frc.robot.commands;

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
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble() * Constants.SwerveDrive.speedMod, Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble() * Constants.SwerveDrive.speedMod, Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble() * Constants.SwerveDrive.speedMod, Constants.stickDeadband);
        brakeCheck = driver.getXButton();
        System.out.println(brakeCheck);
        /* Drive */
        if (brakeCheck == true) {
            translationVal = 0;
            strafeVal = 0;
            rotationVal = 0.05;
        }
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveDrive.maxSpeed), 
            rotationVal * Constants.SwerveDrive.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true);
        
    }
}