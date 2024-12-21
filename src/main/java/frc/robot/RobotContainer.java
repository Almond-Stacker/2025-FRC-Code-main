package frc.robot;

import javax.management.InstanceNotFoundException;

import org.ejml.data.ZMatrix;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver0 = new CommandXboxController(0);
    private final CommandXboxController driver1 = new CommandXboxController(1);

    
    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(new TeleopSwerve(
            s_Swerve, 
            () -> -driver0.getLeftY(), 
            () -> -driver0.getLeftX(), 
            () -> -driver0.getRightX(),
            () -> driver0.y().getAsBoolean()));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        driver0.rightBumper().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }
}
