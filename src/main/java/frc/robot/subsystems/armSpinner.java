package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVenom;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.SpecialFunctions;

public class armSpinner extends SubsystemBase {
     PWMVenom armPwmVenom;

    public void setArmMotor(double speed){
        armPwmVenom.set(speed);
    }

    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
  }


}
