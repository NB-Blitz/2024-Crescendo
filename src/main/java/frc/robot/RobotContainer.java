// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ManipulatorSubsystem m_robotManipulator = new ManipulatorSubsystem();
    // The driver's controller
    Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    Joystick m_manipController = new Joystick(OIConstants.kManipControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    0.3 * -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                    0.3 * -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                    0.5 * -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kTwistDeadband),
                    true, true),
                m_robotDrive));

        m_robotManipulator.setDefaultCommand(
            new RunCommand(
                () -> m_robotManipulator.run(
                    MathUtil.applyDeadband(m_manipController.getY(), OIConstants.kDriveDeadband),
                    MathUtil.applyDeadband(m_manipController.getRawAxis(3), OIConstants.kDriveDeadband)),
                m_robotManipulator));

        // Add default command for the Manipulator
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link Joystick} or {@link edu.wpi.first.wpilibj.XboxController}), and
     * then passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_driverController, OIConstants.kDriveBrakeButton)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive));

        new JoystickButton(m_driverController, OIConstants.kDriveGyroResetButton)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));

        new JoystickButton(m_manipController, 2)
            .whileTrue(new RunCommand(
                () -> m_robotManipulator.stopButtonHandler(),
                m_robotManipulator));

        new JoystickButton(m_manipController, 3)
            .whileTrue(new RunCommand(
                () -> m_robotManipulator.intakeButtonHandler(),
                m_robotManipulator));

        new JoystickButton(m_manipController, 4)
            .whileTrue(new RunCommand(
                () -> m_robotManipulator.outputButtonHandler(),
                m_robotManipulator));

        /*m_manipController.a()
            .whileTrue(new RunCommand(
                () -> m_robotManipulator.ampShootPositionButtonHandler(),
                m_robotManipulator));

        m_manipController.b()
            .whileTrue(new RunCommand(
                () -> m_robotManipulator.intakePositionButtonHandler(),
                m_robotManipulator));

        m_manipController.leftTrigger(0.5)
            .whileTrue(new RunCommand(
                () -> m_robotManipulator.intakeButtonHandler(),
                m_robotManipulator));

        m_manipController.rightBumper()
            .whileTrue(new RunCommand(
                () -> m_robotManipulator.loadPositionButtonHandler(),
                m_robotManipulator));

        m_manipController.rightTrigger(0.5)
            .whileTrue(new RunCommand(
                () -> m_robotManipulator.shootButtonHandler(),
                m_robotManipulator));*/
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
