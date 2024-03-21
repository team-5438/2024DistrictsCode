package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpeakerSubsystem;

public class IntakeCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private SpeakerSubsystem speakerSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, SpeakerSubsystem speakerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.speakerSubsystem = speakerSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        speakerSubsystem.feedMotor.set(0.6);
        intakeSubsystem.intakeMotor.set(1);
    }

    @Override
    public boolean isFinished() {
        /* end command when we have a note */
        return speakerSubsystem.hasNote;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.intakeMotor.set(0);
        speakerSubsystem.feedMotor.set(0);
    }
}
