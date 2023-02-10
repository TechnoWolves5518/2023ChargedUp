package frc.robot;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.15;

    public static final class SwerveDrive {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.875); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(24.875); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2)
      );

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.064686/1); //TODO: This must be tuned to specific robot
        public static final double driveKV = (2.4071);
        public static final double driveKA = (0.066756);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot
        //speed modifier
        public static final double speedMod = 0.5;
        public static final double balanceSpeedMod = 0.4;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        public static final class SpecialFunctions {
            
            // ArmSpinner
            public static int armOne = 14;
            public static int armTwo = 15;
            public static int armThree = 16;
            public static final double bP = 1;
    
            public static double spinMaxVelocity = 0;
            public static double spinMaxAcceleration = 0;
            
            //These are fake gains; in actuality these must be determined individually for each robot
            public static final double bSVolts = 1;
            public static final double bGVolts = 1;

            public static final double bMaxVelocityRadPerSecond = 3;
            public static final double bMaxAccelerationRadPerSecSquared = 10;
            public static final double bVVoltSecondPerRad = 0.5;
            public static final double bAVoltSecondSquaredPerRad = 0.1;

            // The offset of the arm from the horizontal in its neutral position,
            // measured from the horizontal
            public static double spinOffset = 0;

            public static final int[] bEncoderPorts = new int[] {4, 5};
            public static final int bEncoderPPR = 256;
            public static final double bEncoderDistancePerPulse = 2.0 * Math.PI / bEncoderPPR;
            
            //Solenoids 
            public static final int solendoidRight = 17;
            public static final int solendoidLeft = 18;

            // Arm Extender
            public static int armExtender = 13;
            public static final double kP = 1;

            public static double extendMaxVelocity = 0.1;
            public static double extendMaxAcceleration = 0.1;
   

            // These are fake gains; in actuality these must be determined individually for each robot
            public static final double kSVolts = 1;
            public static final double kGVolts = 1;
            public static final double kVVoltSecondPerRad = 0.5;
            public static final double kAVoltSecondSquaredPerRad = 0.1;

            public static final double kMaxVelocityRadPerSecond = 3;
            public static final double kMaxAccelerationRadPerSecSquared = 10;

            // The offset of the arm from the horizontal in its neutral position,
            // measured from the horizontal
            public static final double extendOffset = 0.1;

            public static final int[] kEncoderPorts = new int[] {4, 5};
            public static final int kEncoderPPR = 256;
            public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;
  }

  public static final class AutoConstants {
    public static final double kAutoTimeoutSeconds = 12;
    public static final double kAutoShootTimeSeconds = 7;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;



            
            
            
        }

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(191.074);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2; //2
            public static final int angleMotorID = 1; //1
            public static final int canCoderID = 3; //3
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(226.66);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(236.16);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(146.16);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final double maxPlatformPositivePitch = 1;
        public static final double maxPlatformNegativePitch = -1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    
    public static final class PathPlannerConstants {
        public static final double kMaxSpeed = 3;
        public static final double kMaxAcceleration = 2;
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
        public static final double maxAngularSpeed = Math.PI;
        public static final double maxAngularAcceleration = Math.PI;

        //ditto
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxSpeed, kMaxAcceleration);
    }
}
