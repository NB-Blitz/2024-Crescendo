// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.IOConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;    

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private ManipulatorSubsystem m_robotManipulator;
    private ClimberSubsystem m_robotClimber;

    // The drivers' controllers
    Joystick m_driverController = new Joystick(IOConstants.kDriverControllerPort);
    CommandXboxController m_manipController;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (DriveConstants.isMAXSwerveModules){
           m_robotManipulator = new ManipulatorSubsystem();
           m_robotClimber = new ClimberSubsystem();
           m_manipController = new CommandXboxController(IOConstants.kManipControllerPort);
        }

        NamedCommands.registerCommand("resetGyro", new InstantCommand(() -> m_robotDrive.setGyroToOdometry(), m_robotDrive));
        NamedCommands.registerCommand("shootSpeaker", m_robotManipulator.shootSpeakerCommand());
        
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getY(), IOConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getX(), IOConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getTwist(), IOConstants.kTwistDeadband),
                    0.5 * (1 + -m_driverController.getRawAxis(IOConstants.kDriveSpeedScalerAxis)),
                    true, false),
                m_robotDrive));

        if (DriveConstants.isMAXSwerveModules) {
            m_robotManipulator.setDefaultCommand(
                new RunCommand(
                    () -> m_robotManipulator.run(
                        IntakeConstants.kArmMaxOutput * -MathUtil.applyDeadband(m_manipController.getLeftY(), IOConstants.kDriveDeadband)),
                    m_robotManipulator));

            m_robotClimber.setDefaultCommand(
                new RunCommand(
                    () -> m_robotClimber.move(
                        ClimberConstants.kClimbArmSpeed * -MathUtil.applyDeadband(m_manipController.getRightY(), IOConstants.kDriveDeadband)),
                    m_robotClimber));
        }

        // Add default command for the Manipulator
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("auto_chooser", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link Joystick} or {@link edu.wpi.first.wpilibj.XboxController}), and
     * then passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_driverController, IOConstants.kDriveBrakeButton)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive));

        new JoystickButton(m_driverController, IOConstants.kDriveGyroResetButton)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));
        
        new JoystickButton(m_driverController, IOConstants.kAimSpeakerButton)
            .whileTrue(new RepeatCommand(new InstantCommand(
                () -> m_robotDrive.aimSpeaker(),
                m_robotDrive)));

        if (DriveConstants.isMAXSwerveModules){
            m_manipController.a()
                .whileTrue(new RunCommand(
                    () -> m_robotManipulator.ampShootPositionButtonHandler(),
                    m_robotManipulator));

            m_manipController.b()
                .whileTrue(new RunCommand(
                    () -> m_robotManipulator.stopButtonHandler(),
                    m_robotManipulator));

            m_manipController.x()
                .whileTrue(new RunCommand(
                    () -> m_robotManipulator.shootButtonHandler(false),
                    m_robotManipulator));

            m_manipController.leftBumper()
                .whileTrue(new RunCommand(
                    () -> m_robotManipulator.intakePositionButtonHandler(),
                    m_robotManipulator));

            m_manipController.rightBumper()
                .whileTrue(new RunCommand(
                    () -> m_robotManipulator.loadPositionButtonHandler(),
                    m_robotManipulator));

            m_manipController.leftTrigger(0.5)
                .whileTrue(new RunCommand(
                    () -> m_robotManipulator.intakeButtonHandler(),
                    m_robotManipulator));

            m_manipController.rightTrigger(0.5)
                .whileTrue(new RunCommand(
                    () -> m_robotManipulator.shootButtonHandler(true),
                    m_robotManipulator));

            m_manipController.rightStick()
                .onTrue(new InstantCommand(
                    () -> m_robotManipulator.setDebugMode(true),
                    m_robotManipulator))
                .onFalse(new InstantCommand(
                    () -> m_robotManipulator.setDebugMode(false),
                    m_robotManipulator));

            m_manipController.povLeft()
                .whileTrue(new RunCommand(
                    () -> m_robotManipulator.resetEncoder(),
                    m_robotManipulator));

            m_manipController.povUp()
                .whileTrue(new RunCommand(
                    () -> m_robotManipulator.usePositionBounds(true),
                    m_robotManipulator));

            m_manipController.povDown()
                .whileTrue(new RunCommand(
                    () -> m_robotManipulator.usePositionBounds(false),
                    m_robotManipulator));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // Might use this later for autonomousPeriodic in Robot.java
    // public void updateOdometry() {
    //     m_robotDrive.updateOdometry();
    // }
}