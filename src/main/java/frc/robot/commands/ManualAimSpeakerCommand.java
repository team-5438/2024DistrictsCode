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

        addRequirements(speakerSubsystem);
    }

    @Override
    public void execute() {
        pivotSpeed = MathUtil.applyDeadband(-operator.getRightY(), Constants.Operator.rightStick.Y);
        pivotSpeed = MathUtil.clamp(pivotSpeed, -Constants.Shooter.Speaker.maxPivotSpeed, Constants.Shooter.Speaker.maxPivotSpeed);

        /* we use a feed forward when the shooter is below a certain value to keep it from falling
         * the reason we do this under a certain range is because the steel gears on their own are
         * able to hold up the shooter without feedforward up to this position
         */
        if (speakerSubsystem.pivotEncoderDistance < 0.116) {
            pivotSpeed += speakerSubsystem.pivotFeedforward.calculate(speakerSubsystem.pivotEncoderDistance, pivotSpeed);
        }

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