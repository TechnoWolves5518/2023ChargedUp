// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;


/** Add your docs here. */
public class armGripper extends SubsystemBase{
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    

    Solenoid leftPistSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, SpecialFunctions.solendoidLeft);
    Solenoid rightPistSolenoid = new Solenoid(PneumaticsModuleType.REVPH, SpecialFunctions.solendoidRight);


    public void theGrip() {
        leftPistSolenoid.set(true);
        leftPistSolenoid.set(false);

        rightPistSolenoid.set(true);
        rightPistSolenoid.set(false);}
    
    
    @Override
    public void periodic(){}
    
}
