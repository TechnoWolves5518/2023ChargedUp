package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class armExtender extends SubsystemBase{
    static TalonSRX armExtender;
    public armExtender() {
        armExtender = new TalonSRX(SpecialFunctions.armExtender);
    }

    public static void setMotors(TalonSRXControlMode Position, double speed){
        armExtender.set(Position, speed);
    }
    
    
    @Override
    public void periodic(){}
}