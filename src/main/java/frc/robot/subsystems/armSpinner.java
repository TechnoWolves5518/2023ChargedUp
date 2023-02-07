package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class armSpinner extends SubsystemBase {
    static CANVenom armPwmVenomOne;
    static CANVenom armPwmVenomTwo;
    static CANVenom armPwmVenomThree;

    public armSpinner() {
        armPwmVenomOne = new CANVenom(SpecialFunctions.armPwmVenomOne);
        armPwmVenomTwo = new CANVenom(SpecialFunctions.armPwmVenomTwo);
        armPwmVenomThree = new CANVenom(SpecialFunctions.armPwmVenomThree);
    }

    public static void setMotors(double speed){
       armPwmVenomOne.set(speed);
       armPwmVenomTwo.set(speed);
       armPwmVenomThree.set(speed);
    }

    @Override
    public void periodic(){}
}