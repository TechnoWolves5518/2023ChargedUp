package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVenom;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armMotor extends SubsystemBase {
    // Motor definitions 
    private PWMVenom armPwmVenom = new PWMVenom(13);
    


    public void setArmMotor(double speed){
        armPwmVenom.set(speed);
    }






}
