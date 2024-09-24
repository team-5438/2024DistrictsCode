package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SpeakerSubsystem;
import frc.robot.subsystems.AmpSubsystem;

public class AmpPresetCommand extends Command {
    private SpeakerSubsystem speakerSubsystem;
    private AmpSubsystem ampSubsystem;

    public AmpPresetCommand(SpeakerSubsystem speakerSubsystem, AmpSubsystem ampSubsystem) {
        this.speakerSubsystem = speakerSubsystem;
        this.ampSubsystem = ampSubsystem;
        /* override default aiming */
        addRequirements(speakerSubsystem);
    }

    @Override
    public void execute() {
        speakerSubsystem.pivotMotor.set(speakerSubsystem.pivotPID.calculate(speakerSubsystem.pivotEncoderDistance, 0.12));
    }

    @Override
    public boolean isFinished() {
        if (ampSubsystem.pivotEncoder.getPosition() < 0.1) {
            return true;
        }
        return false;
    }
}