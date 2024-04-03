package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpeakerSubsystem;

public class FeedCommand extends Command {
    private SpeakerSubsystem speakerSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private double speed;

    public FeedCommand(SpeakerSubsystem speakerSubsystem, IntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speakerSubsystem = speakerSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        speakerSubsystem.feedMotor.set(speed);
        intakeSubsystem.intakeMotor.set(0.2);
    }

    @Override
    public boolean isFinished() {
        /* end command when we no longer have a note */
        return !speakerSubsystem.hasNote && DriverStation.isAutonomous();
    }

    @Override
    public void end(boolean interrupted) {
        speakerSubsystem.feedMotor.set(0);
        intakeSubsystem.intakeMotor.set(0);
    }
}