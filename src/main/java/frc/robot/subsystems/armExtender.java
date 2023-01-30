package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class armExtender extends SubsystemBase{
    
    public static CANSparkMax armViagra;

    public void extendSystem(double speed){
        armViagra.set(speed);
    }
    

}