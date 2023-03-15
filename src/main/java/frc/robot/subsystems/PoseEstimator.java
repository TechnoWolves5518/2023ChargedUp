
package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;


public class PoseEstimator extends SubsystemBase{
    private final PhotonCamera camera = CameraConstants.camera;
    final double CameraHeight = CameraConstants.cameraHeightMeters;
    final double TargetHeight = CameraConstants.scoringAprilTagHeightMeters;
    final double cameraPitchRadians = CameraConstants.cameraAngleRadians;
    public boolean hasTargets = false;
    public boolean isTargeting = true;
   
    private double targetAngle = CameraConstants.targetAngle;
    private double range;
    double forwardSpeed;
    double x_pitch = CameraConstants.xPitch;

      public boolean hasTargets() {
        return hasTargets;
      }
       
      public double getTargetAngle() {
        return targetAngle;
      }

      public void pipelineIndex() {
        camera.setPipelineIndex(0);
      }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        camera.setDriverMode(false);

        if (result.hasTargets()) {
            hasTargets = true;
            targetAngle = result.getBestTarget().getPitch(); //pitch or yaw?
            range = PhotonUtils.calculateDistanceToTargetMeters(
                    CameraHeight,
                    TargetHeight,
                    cameraPitchRadians ,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
          } else {
            hasTargets = false;
            range = -2;
            //targetAngle = -1;
          }

        SmartDashboard.putBoolean("Has target", hasTargets);   
        SmartDashboard.putNumber("Distance between target", range);   
    }


}