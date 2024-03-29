package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class Intake extends Command {
    ManipulatorSubsystem manipulatorSubsystem;

    public Intake(ManipulatorSubsystem manipulatorSubsystem) {
        this.manipulatorSubsystem = manipulatorSubsystem;
    }

    @Override
    public void initialize() {
        manipulatorSubsystem.intakeButtonHandler();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
