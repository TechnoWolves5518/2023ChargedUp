package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    //define swerve variables
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private double previousAngle;
    public Swerve() {
        //Gyro variable definitions
        gyro = new Pigeon2(Constants.SwerveDrive.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        previousAngle = getElevationAngle();

        //define all the swerve modules and assign their constants
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveDrive.Mod0.constants),
            new SwerveModule(1, Constants.SwerveDrive.Mod1.constants),
            new SwerveModule(2, Constants.SwerveDrive.Mod2.constants),
            new SwerveModule(3, Constants.SwerveDrive.Mod3.constants)
        };
        //delay the cancoder calibration to prevent offset not working as intended
        Timer.delay(1.0);
        resetModulesToAbsolute();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveDrive.swerveKinematics, getYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            DriverStation.reportError("CANcoder on Module " + mod.moduleNumber + " took " + mod.CANcoderInitTime + " ms to be ready.", false);
        }


    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveDrive.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveDrive.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveDrive.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }
    public double getvisionheading(){
        return -gyro.getYaw();
      }
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }
    //yaw value for field oriented driving
    public Rotation2d getYaw() {
        return (Constants.SwerveDrive.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    //grab the angle for use for leveling the bot
    public double getElevationAngle() {
        return gyro.getPitch();
    }
    //grab the angle to ensure the bot drives straight on the station, untested
    public double getChargeOffset() {
        return gyro.getRoll();
    }
    public double getRawYaw() {
        return gyro.getYaw();
    }
    @Override
    //values for debugging
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  
        
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        previousAngle = getElevationAngle();
        SmartDashboard.putNumber("balance angle: ", previousAngle);
    }
}