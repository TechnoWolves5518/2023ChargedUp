package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.SpecialFunctions;

public class armSpinner extends SubsystemBase {

    public static void spinSystem(double speed){
        SpecialFunctions.armPwmVenom.set(speed);
    }


    @Override
    public void periodic(){}
}
