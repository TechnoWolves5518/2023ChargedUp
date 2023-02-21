// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;


/** Add your docs here. */
public class armGripper extends SubsystemBase{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public static DoubleSolenoid gripSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
                                                            SpecialFunctions.solenoidOn, 
                                                            SpecialFunctions.solenoidOff);
                    

    public static void closeHand(){
        gripSolenoid.set(DoubleSolenoid.Value.kForward);
        gripSolenoid.toggle();
    }


    public static void openHand(){
        gripSolenoid.set(DoubleSolenoid.Value.kReverse);
        gripSolenoid.toggle();
    }

    @Override
    public void periodic(){

    }
}