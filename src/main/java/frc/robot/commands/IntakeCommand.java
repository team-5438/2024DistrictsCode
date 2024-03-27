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
    private int[][] greenFlash = {{0,255,0,1000},{0,0,0,1000},{0,255,0,1000},{0,0,0,1000},{0,255,0,1000},{0,0,0,1000}};

    public IntakeCommand(IntakeSubsystem intakeSubsystem, SpeakerSubsystem speakerSubsystem, LEDSubsystem ledSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.speakerSubsystem = speakerSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.strip0.solidColorRGB(255, 0, 255);
        ledSubsystem.strip0.set();
        speakerSubsystem.feedMotor.set(0.1);
        intakeSubsystem.intakeMotor.set(0.9);
    }

    @Override
    public boolean isFinished() {
        /* end command when we have a note */
        if(speakerSubsystem.hasNote){
            new FlashLEDS(ledSubsystem.strip0, greenFlash);
        } 
        
        return speakerSubsystem.hasNote;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.intakeMotor.set(0);
        speakerSubsystem.feedMotor.set(0);

        ledSubsystem.strip0.setDefaultLED();
    }
}
