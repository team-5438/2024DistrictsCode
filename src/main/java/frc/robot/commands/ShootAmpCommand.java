package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.SpeakerSubsystem;

public class ShootAmpCommand extends Command {
    private SpeakerSubsystem speakerSubsystem;

    public ShootAmpCommand(SpeakerSubsystem speakerSubsystem) {
        this.speakerSubsystem = speakerSubsystem;
    }

    @Override
    public void initialize() {
        speakerSubsystem.topShootMotor.set(-0.05);
        speakerSubsystem.bottomShootMotor.set(0.55);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        speakerSubsystem.topShootMotor.set(0);
        speakerSubsystem.bottomShootMotor.set(0);
    }
}
