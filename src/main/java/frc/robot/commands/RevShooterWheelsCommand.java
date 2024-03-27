package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SpeakerSubsystem;

public class RevShooterWheelsCommand extends Command {
    private SpeakerSubsystem speakerSubsystem;
    private double speed;

    public RevShooterWheelsCommand(SpeakerSubsystem speakerSubsystem, double speed) {
        this.speakerSubsystem = speakerSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        speakerSubsystem.topShootMotor.set(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        speakerSubsystem.topShootMotor.set(0);
    }
}
