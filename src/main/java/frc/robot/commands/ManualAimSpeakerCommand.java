package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.SpeakerSubsystem;

public class ManualAimSpeakerCommand extends Command {
    private SpeakerSubsystem speakerSubsystem;
    private PS4Controller operator;
    private double pivotSpeed;

    public ManualAimSpeakerCommand(SpeakerSubsystem speakerSubsystem, PS4Controller operator) {
        this.speakerSubsystem = speakerSubsystem;
        this.operator = operator;
    }

    @Override
    public void execute() {
        pivotSpeed = MathUtil.applyDeadband(-operator.getRightY(), Constants.Operator.rightStick.Y);
        pivotSpeed = MathUtil.clamp(pivotSpeed, -Constants.Shooter.Speaker.maxPivotSpeed, Constants.Shooter.Speaker.maxPivotSpeed);

        speakerSubsystem.pivotMotor.set(pivotSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        speakerSubsystem.pivotMotor.set(0);
    }
}
