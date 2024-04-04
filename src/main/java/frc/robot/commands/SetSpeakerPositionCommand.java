package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SpeakerSubsystem;

public class SetSpeakerPositionCommand extends Command {
    private SpeakerSubsystem speakerSubsystem;
    private double encoderPosition;

    public SetSpeakerPositionCommand(SpeakerSubsystem speakerSubsystem, double encoderPosition) {
        this.speakerSubsystem = speakerSubsystem;
        this.encoderPosition = encoderPosition;
    }
    
    @Override
    public void execute() {
        speakerSubsystem.pivotMotor.set(speakerSubsystem.pivotPID.calculate(speakerSubsystem.pivotEncoderDistance, encoderPosition));
    }

    @Override
    public boolean isFinished() {
        double aimError = Math.abs(speakerSubsystem.pivotEncoderDistance - encoderPosition);
        if (aimError <= Constants.Shooter.Speaker.aimedTolerance) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        speakerSubsystem.pivotMotor.set(0);
    }
}
