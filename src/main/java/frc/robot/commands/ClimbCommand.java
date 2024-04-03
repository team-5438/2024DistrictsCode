package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {
    private ClimberSubsystem climberSubsystem;
    private double speed;

    public ClimbCommand(ClimberSubsystem climberSubsystem, double speed) {
        this.climberSubsystem = climberSubsystem;
        this.speed = speed;

        addRequirements(climberSubsystem);
    } 

    @Override
    public void initialize() {
        climberSubsystem.LClimber.set(speed);
        climberSubsystem.RClimber.set(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.LClimber.set(0);
        climberSubsystem.RClimber.set(0);
    }
}