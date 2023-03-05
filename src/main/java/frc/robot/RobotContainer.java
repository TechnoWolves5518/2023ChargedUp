package frc.robot;

import java.sql.Driver;

import javax.lang.model.element.ModuleElement.DirectiveVisitor;

import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.AutoCommands.*;
import frc.robot.commands.*;
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

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton testButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton testButton2 = new JoystickButton(driver, XboxController.Button.kB.value);
    
    /* Special Buttons */
    private final JoystickButton specialGripper = new JoystickButton(special, XboxController.Button.kB.value);
    private final JoystickButton specialDownButton = new JoystickButton(special, XboxController.Button.kY.value);
    private final JoystickButton specialUpButton = new JoystickButton(special, XboxController.Button.kA.value);
    private final JoystickButton specialExtend = new JoystickButton(special, XboxController.Button.kRightBumper.value);
    private final JoystickButton specialRetract = new JoystickButton(special, XboxController.Button.kLeftBumper.value);
    private final JoystickButton specialIn = new JoystickButton(special, XboxController.Button.kStart.value);
    private final JoystickButton specialOut = new JoystickButton(special, XboxController.Button.kBack.value);
    private final JoystickButton specialHandToggle = new JoystickButton(special, XboxController.Button.kRightStick.value);

    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final AutoSelector autoSelector;
    private final ArmExtender a_ArmExtender = new ArmExtender();
    private final ArmSpinner a_Spinner = new ArmSpinner();
    private final TestSRX t_test = new TestSRX();
    private final HandSpinner h_spinner = new HandSpinner();
    private final HandGripper h_grip = new HandGripper();
    private final Compressor c_Compressor = new Compressor();
    private final BrakeArm b_arm = new BrakeArm();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );


        // Configure the button bindings
        autoSelector = new AutoSelector(s_Swerve);
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
        testButton.onTrue(new HandToggle(h_grip));
        
        //ShmoButtons
        specialIn.whileTrue(new PullIn(h_spinner));
        specialOut.whileTrue(new PushOut(h_spinner));
        specialHandToggle.onTrue(new HandToggle(h_grip));
        

        


        
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
