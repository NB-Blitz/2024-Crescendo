package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ShootSpeaker extends Command {
    ManipulatorSubsystem manipulatorSubsystem;

    Timer timer;

    public ShootSpeaker(ManipulatorSubsystem manipulatorSubsystem) {
        this.manipulatorSubsystem = manipulatorSubsystem;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        manipulatorSubsystem.shootButtonHandler(false);
        if (timer.get() >= 2) {
            manipulatorSubsystem.shootButtonHandler(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        manipulatorSubsystem.stopButtonHandler();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 3;
    }
}
