package frc.lib.util;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int turnMotorID;
    public final int cancoderID;
    public final double angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param turnMotorID
     * @param canCoderID
     * @param turnOffset
     */
    public SwerveModuleConstants(int driveMotorID, int turnMotorID, int canCoderID, double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
