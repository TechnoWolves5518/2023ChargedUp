// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /// Button Ports ///
    public static int yButton = 4;
    public static int xButton = 3;
    public static int aButton = 1;
    public static int bButton = 2;

    public static int leftStickX = 0;
    public static int leftStickY = 1;
    public static int rightStickX = 4;
    public static int rightStickY = 5;
    public static int leftTrigger = 2;
    public static int rightTrigger = 3;

    public static int rightBumper = 6;
    public static int leftBumper = 5;
    public static int leftStickButton = 9;
    public static int rightStickButton = 10;
    public static int startButton = 8;
    public static int backButton = 7;
    //deadzoen value
    public static double deadzone = 0.15;
}