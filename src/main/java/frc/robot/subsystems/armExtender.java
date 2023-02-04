package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class armExtender extends SubsystemBase{
    static CANSparkMax armViagra;
    public armExtender() {
        armViagra = new CANSparkMax(SpecialFunctions.armViagra, MotorType.kBrushed);
    }

    public static void setMotors(double speed){
        armViagra.set(speed);
    }
    
    @Override
    public void periodic(){}
}