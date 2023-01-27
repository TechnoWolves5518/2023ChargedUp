package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class armExtender extends SubsystemBase{
    
    private CANSparkMax moter1 = new CANSparkMax(13, MotorType.kBrushed);


}
