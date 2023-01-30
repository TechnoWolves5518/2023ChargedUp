package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.SpecialFunctions;

public class armExtender extends SubsystemBase{


    public void extendSystem(double speed){
        SpecialFunctions.armViagra.set(speed);
    }
    

}