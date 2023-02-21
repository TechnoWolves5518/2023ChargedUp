// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;


/** Add your docs here. */
public class armGripper extends SubsystemBase{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public static DoubleSolenoid leftGrip = new DoubleSolenoid(PneumaticsModuleType.REVPH, SpecialFunctions.leftsolenoidClose, SpecialFunctions.leftsolenoidOpen);
    public static DoubleSolenoid rightGrip = new DoubleSolenoid(PneumaticsModuleType.REVPH,SpecialFunctions.leftsolenoidClose, SpecialFunctions.leftsolenoidOpen);
                                                           
                                                          
                                                          
    /** Creates a new ShooterSubsystem. */

    public armGripper() {
        leftGrip.set(Value.kReverse);
        rightGrip.set(Value.kReverse);}
                                                          
    public static void closeHand(){
        leftGrip.set(Value.kForward);
        rightGrip.set(Value.kForward);
        }
                                                          
    public static void openHand(){
        leftGrip.set(Value.kReverse);
        rightGrip.set(Value.kReverse);
    }
                                    

}