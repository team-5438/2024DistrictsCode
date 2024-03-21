package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.SpeakerSubsystem;

public class RevShooterWheels extends Command {
    private AmpSubsystem ampSubsystem;
    private SpeakerSubsystem speakerSubsystem;
    private double speed;

    public AmpShootCommand(AmpSubsystem ampSubsystem, SpeakerSubsystem speakerSubsystem, double speed) {
        this.ampSubsystem = ampSubsystem;
        this.speakerSubsystem = speakerSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        ampSubsystem.shootMotor.set(speed);
        speakerSubsystem.topShootMotor.set(0.2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        ampSubsystem.shootMotor.set(0);
        speakerSubsystem.topShootMotor.set(0);
    }
}
