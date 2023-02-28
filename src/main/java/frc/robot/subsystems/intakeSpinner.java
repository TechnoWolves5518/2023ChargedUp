// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class intakeSpinner extends SubsystemBase {

    public static CANSparkMax leftIntake = new CANSparkMax(SpecialFunctions.gripLeftIntake, MotorType.kBrushless);
    public static CANSparkMax rightIntake = new CANSparkMax(SpecialFunctions.gripRightIntake, MotorType.kBrushless);
    
        

        public static void pullIn(){
            rightIntake.follow(leftIntake, false);
            leftIntake.set(- SpecialFunctions.intakeSpeed);
        }

        public static void pushOut(){
            rightIntake.follow(leftIntake, false);
            leftIntake.set(SpecialFunctions.intakeSpeed);}
            
        public static void endIntake(){
            rightIntake.follow(leftIntake, false);
            leftIntake.set(0);

        }
    
    @Override
    public void periodic() {}

}
