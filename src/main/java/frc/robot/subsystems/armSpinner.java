package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVenom;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armSpinner extends SubsystemBase {
    public static PWMVenom armPwmVenom;

    public void spinSystem(double speed){
        armPwmVenom.set(speed);
    }


    @Override
    public void periodic(){}
}
