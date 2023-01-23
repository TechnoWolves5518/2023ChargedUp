package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVenom;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class armMotor extends SubsystemBase {
    private PWMVenom armPwmVenom = new PWMVenom(0); // TODO: Change this

    public void setArmMotor(double speed){
        armPwmVenom.set(speed);
    }




}
