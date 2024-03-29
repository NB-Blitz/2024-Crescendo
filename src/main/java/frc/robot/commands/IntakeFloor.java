package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem.ARM_POS;

public class IntakeFloor extends Command {
    ManipulatorSubsystem manipulatorSubsystem;

    public IntakeFloor(ManipulatorSubsystem manipulatorSubsystem) {
        this.manipulatorSubsystem = manipulatorSubsystem;
    }

    @Override
    public void initialize() {
        manipulatorSubsystem.floorPositionButtonHandler();
    }

    @Override
    public boolean isFinished() {
        return manipulatorSubsystem.currentPos == ARM_POS.FLOOR;
    }
}
