package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpeakerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utils.FlashLEDS;

public class IntakeCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private SpeakerSubsystem speakerSubsystem;
    private LEDSubsystem ledSubsystem;
    private int[][] greenFlash = {{0,255,0,100},{0,0,0,100},{0,255,0,100},{0,0,0,100},{0,255,0,100},{0,0,0,100}};

    public IntakeCommand(IntakeSubsystem intakeSubsystem, SpeakerSubsystem speakerSubsystem, LEDSubsystem ledSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.speakerSubsystem = speakerSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.setFire();
        if (speakerSubsystem.pivotEncoderDistance > 0.1) {
            speakerSubsystem.feedMotor.set(0);
        } else {
            speakerSubsystem.feedMotor.set(0.2);
        }
        intakeSubsystem.intakeMotor.set(1);
    }

    @Override
    public boolean isFinished() {
        /* end command when we have a note */
        if (speakerSubsystem.hasNote) {
            ledSubsystem.setForestTinkle();
        } 
        
        return speakerSubsystem.hasNote;
    }

    @Override
    public void end(boolean interrupted) {
        speakerSubsystem.feedMotor.set(0);
        intakeSubsystem.intakeMotor.set(0);

        ledSubsystem.setDefault();
    }
}