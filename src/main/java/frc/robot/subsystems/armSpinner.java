package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class armSpinner extends SubsystemBase {
    static CANVenom armPwmVenom;
    public armSpinner() {
        armPwmVenom = new CANVenom(SpecialFunctions.armPwmVenom);
    }
    public static void setMotors(double speed){
       armPwmVenom.set(speed);
    }


    @Override
    public void periodic(){}
}
