package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.AutoSelector;
import frc.robot.autos.AutoDriveBase.AutoBalance;
import frc.robot.commands.ArmExtender.ExtendArm;
import frc.robot.commands.ArmExtender.TestRetract;
import frc.robot.commands.DriveBase.DpadDriveBack;
import frc.robot.commands.DriveBase.DpadDriveForward;
import frc.robot.commands.DriveBase.DpadDriveLeft;
import frc.robot.commands.DriveBase.DpadDriveRight;
import frc.robot.commands.DriveBase.TeleopSwerve;
import frc.robot.commands.Hand.HandToggle;
import frc.robot.commands.Hand.PullIn;
import frc.robot.commands.Hand.PushOut;
import frc.robot.commands.MiscellaneousCommands.LEDOneToggle;
import frc.robot.commands.MiscellaneousCommands.LEDThreeToggle;
import frc.robot.commands.MiscellaneousCommands.LEDTwoToggle;
//import frc.robot.commands.PhotonVision.AutoAlign;
import frc.robot.commands.armRotator.GoToDefaultState;
import frc.robot.commands.armRotator.GoToHopper;
import frc.robot.commands.armRotator.GoToPassiveStage;
import frc.robot.commands.armRotator.GoToPickup;
//import frc.robot.commands.armRotator.GoToStageOne;
import frc.robot.commands.armRotator.GoToStageTwo;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick special = new Joystick(1); 
    private final Joystick debug = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverBalance = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverToggleClaw = new JoystickButton(driver, XboxController.Button.kB.value);
    private final POVButton driverForward = new POVButton(driver, 0);
    private final POVButton driverRight = new POVButton(driver, 90);
    private final POVButton driverBack = new POVButton(driver, 180);
    private final POVButton driverLeft = new POVButton(driver, 270);
    
    /* Special Buttons */
    private final JoystickButton specialGripper = new JoystickButton(special, XboxController.Button.kB.value);
    private final JoystickButton specialDefualtState = new JoystickButton(special, XboxController.Button.kA.value);
    private final JoystickButton specialExtend = new JoystickButton(special, XboxController.Button.kRightBumper.value);
    private final JoystickButton specialStageTwo = new JoystickButton(special, XboxController.Button.kX.value);
    private final JoystickButton specialRetract = new JoystickButton(special, XboxController.Button.kLeftBumper.value);
    private final JoystickButton specialIn = new JoystickButton(special, XboxController.Button.kStart.value);
    private final JoystickButton specialOut = new JoystickButton(special, XboxController.Button.kBack.value);
    private final JoystickButton specialStageOne = new JoystickButton(special, XboxController.Button.kY.value);
    private final JoystickButton specialPassive = new JoystickButton(special, XboxController.Button.kRightStick.value);
    private final POVButton specialHopper = new POVButton(special, 0);

    //debug button
    private final JoystickButton debugButton = new JoystickButton(debug, XboxController.Button.kA.value);
    private final JoystickButton debugButton2 = new JoystickButton(debug, XboxController.Button.kB.value);
    private final JoystickButton debugButton3 = new JoystickButton(debug, XboxController.Button.kX.value);

    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final AutoSelector autoSelector;
    private final ArmExtender a_ArmExtender = new ArmExtender();
    private final ArmSpinner a_Spinner = new ArmSpinner();
    private final TestSRX t_test = new TestSRX();
    private final HandSpinner h_spinner = new HandSpinner();
    private final HandGripper h_grip = new HandGripper();
    //private final Compressor c_Compressor = new Compressor();
    private final BrakeArm b_arm = new BrakeArm();
    //private final Vision p_Estimator = new Vision();
    private final LEDControl l_Control = new LEDControl();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );


        // Configure the button bindings
        autoSelector = new AutoSelector(s_Swerve, h_grip, a_Spinner, b_arm, h_spinner, a_ArmExtender);
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driverBalance.whileTrue(new AutoBalance(s_Swerve));
        driverToggleClaw.onTrue(new HandToggle(h_grip));
        driverForward.whileTrue(new DpadDriveForward(s_Swerve));
        driverRight.whileTrue(new DpadDriveRight(s_Swerve));
        driverBack.whileTrue(new DpadDriveBack(s_Swerve));
        driverLeft.whileTrue(new DpadDriveLeft(s_Swerve));
        //alignRobot.whileTrue(new AutoAlign(s_Swerve, p_Estimator, () -> -driver.getRawAxis(translationAxis), () -> -driver.getRawAxis(strafeAxis)));

        //ShmoButtons
        specialIn.whileTrue(new PullIn(h_spinner));
        specialOut.whileTrue(new PushOut(h_spinner));
        specialGripper.onTrue(new HandToggle(h_grip));
        specialExtend.whileTrue(new ExtendArm(a_ArmExtender));
        specialRetract.whileTrue(new TestRetract(t_test));
        specialDefualtState.onTrue(new GoToDefaultState(a_Spinner, b_arm, a_ArmExtender, h_grip));
        specialStageOne.onTrue(new GoToPickup(a_Spinner, b_arm));
        specialStageTwo.onTrue(new GoToStageTwo(a_Spinner, b_arm));
        specialPassive.onTrue(new GoToPassiveStage(a_Spinner, b_arm, a_ArmExtender));
        specialHopper.onTrue(new GoToHopper(a_Spinner, b_arm));

        //debug buttons
        debugButton.toggleOnTrue(new LEDOneToggle(l_Control));
        debugButton2.toggleOnTrue(new LEDTwoToggle(l_Control));
        debugButton3.toggleOnTrue(new LEDThreeToggle(l_Control));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoSelector.getSelected();
    }
}
