package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpeakerSubsystem;

public class FeedCommand extends Command {
    private SpeakerSubsystem speakerSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public FeedCommand(SpeakerSubsystem speakerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.speakerSubsystem = speakerSubsystem;
    }

    @Override
    public void initialize() {
        speakerSubsystem.feedMotor.set(1);
        intakeSubsystem.intakeMotor.set(1);
    }

    @Override
    public boolean isFinished() {
        /* end command when we no longer have a note */
        return !speakerSubsystem.hasNote;
    }

    @Override
    public void end(boolean interrupted) {
        speakerSubsystem.feedMotor.set(0);
        intakeSubsystem.intakeMotor.set(0);
    }
}
