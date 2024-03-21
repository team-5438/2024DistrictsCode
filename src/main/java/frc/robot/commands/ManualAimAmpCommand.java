package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.AmpSubsystem;

public class ManualAimAmpCommand extends Command {
    private AmpSubsystem ampSubsystem;
    private PS4Controller operator;
    private double pivotSpeed;

    public ManualAimAmpCommand(AmpSubsystem ampSubsystem, PS4Controller operator) {
        this.ampSubsystem = ampSubsystem;
        this.operator = operator;
    }

    @Override
    public void execute() {
        pivotSpeed = MathUtil.applyDeadband(operator.getLeftY(), Constants.Operator.leftStick.Y);

        ampSubsystem.pivotMotor.set(pivotSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        ampSubsystem.pivotMotor.set(0);
    }
}
