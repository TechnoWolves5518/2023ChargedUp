//imports
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile; //for autonomous
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;



public class RobotMap {
    /* RobotMap is a file that is where predominanty static variables are defined
       Speedmodifiers, CAN ID's, and deadzone values are declared here
       
       This is to ensure that there is one place to go to to change a variable
       Otherwise we would have to change every version of the varable and lose time that could be spent doing other tasks
    */

    public static final class AutonomousConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double KMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double KMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1.25;
        public static final double kPYController = 1.25;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, KMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class PathWeaverConstants {
        public static final String testPath = "paths/testAuto.wpilib.json";
    }
    //Drivetrain modifiers
    //define invertion of the Gryo
    public static final boolean inverGyro = false;
    //speed modifiers
    public static final double maxSpeed = 4.5; //value in meters per second default is 4.5
    public static final double maxSteeringSpeed = 11.5; //how fast the bot can spin around
    //wheel specs
    public static final double trackWidth = Units.inchesToMeters(24.875); //distance between right and left wheels
    public static final double wheelBase = Units.inchesToMeters(24.875); //distance between front and back wheels.
    public static final double wheelDiameter = Units.inchesToMeters(3.7); //diameter of sweve wheel
    public static final double wheelCircumference = wheelDiameter * Math.PI; //circumference of a swerve wheel
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;
    public static final double driveGearRatio = 6.75/1; //L2 mk4i gear ratio
    public static final double turnGearRatio = (150/7)/1; //standard gear ratio
    
    //swerve Kinematics
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase/2, trackWidth/2),
        new Translation2d(wheelBase/2, -trackWidth/2),
        new Translation2d(-wheelBase/2, trackWidth/2),
        new Translation2d(-wheelBase/2, -trackWidth/2));

    //current limiting
    //turning motor limits
    public static final int turnContinuousCurrentLimit = 25; //amp limits for turn motor
    public static final int turnPeakCurrentLimit = 40;
    public static final double turnPeakCurrentDuration = 0.1; //time in seconds
    public static final boolean turnEnableCurrentLimit = true; //turn off if you want to break something
    //drive motor limits
    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    //turn motor PID Values
    public static final double turnKP = 0.6; //PID values copied from Github
    public static final double turnKI = 0.0;
    public static final double turnKD = 12.0;
    public static final double turnKF = 0.0;

    //drive motor PID values
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    //Drive motor Characterization
    public static final double driveKS = (0.64686/1);
    public static final double driveKV = (2.4071/1);
    public static final double driveKA = (0.066756/1);

    public static final NeutralMode turnNeutralMode = NeutralMode.Brake; //what to do when no input is detected
    public static final NeutralMode driveNeutralMode = NeutralMode.Coast; 

    //motor inversion
    public static final boolean driveMotorInvert = false; //for turning optimization
    public static final boolean turnMotorInvert = false;

    //incoder invert
    public static final boolean canCoderInvert = true;
    //Swerve Module variables and CAN IDs
    //CAN is the system that allows the code to identify motor controllers and in turn 
    // ensure that the correct motor controller is being used for the right task
    
    //front right module
    public static final class Mod0 {
    public static final int frontRightControlMotor = 1;
    public static final int frontRightDriveMotor = 2;
    public static final int frontRightEncoder = 3;
    public static final double angleOffset = 133.24; //how far off the motor is from center
    public static final SwerveModuleConstants constants =
    new SwerveModuleConstants(frontRightDriveMotor, frontRightControlMotor, frontRightEncoder, angleOffset);
    }
    //front left module
    public static final class Mod1{
    public static final int frontLeftControlMotor = 10;
    public static final int frontLeftDriveMotor = 11;
    public static final int frontLeftEncoder = 12;
    public static final double angleOffset = 34.36;
    public static final SwerveModuleConstants constants =
    new SwerveModuleConstants(frontLeftDriveMotor, frontLeftControlMotor, frontLeftEncoder, angleOffset);
    }
    //back right module
    public static final class Mod2{
    public static final int backRightControlMotor = 4;
    public static final int backRightDriveMotor = 5;
    public static final int backRightEncoder = 6;
    public static final double angleOffset = 348.04;
    public static final SwerveModuleConstants constants = 
    new SwerveModuleConstants(backRightDriveMotor, backRightControlMotor, backRightEncoder, angleOffset);
    }
    //back left module
    public static final class Mod3{
    public static final int backLeftControlMotor = 7;
    public static final int backLeftDriveMotor = 8;
    public static final int backLeftEncoder = 9;
    public static final double angleOffset = 119.53;
    public static final SwerveModuleConstants constants =
    new SwerveModuleConstants(backLeftDriveMotor, backLeftControlMotor, backLeftEncoder, angleOffset);    }

    //auto variables
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}
