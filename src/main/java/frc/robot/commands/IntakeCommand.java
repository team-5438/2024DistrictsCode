package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpeakerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utils.LEDStrip;
import frc.robot.utils.FlashLEDS;

public class IntakeCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private SpeakerSubsystem speakerSubsystem;
    private LEDSubsystem ledSubsystem;
    private int[][] greenFlash = {{0,255,0,1000},{0,0,0,1000},{0,255,0,1000},{0,0,0,1000},{0,255,0,1000}};

    public IntakeCommand(IntakeSubsystem intakeSubsystem, SpeakerSubsystem speakerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.speakerSubsystem = speakerSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        speakerSubsystem.feedMotor.set(0.6);
        intakeSubsystem.intakeMotor.set(1);
        ledSubsystem.strip0.solidColorRGB(255, 0, 255);
        ledSubsystem.strip0.set();
    }

    @Override
    public boolean isFinished() {
        /* end command when we have a note */
        if(speakerSubsystem.hasNote){
            new FlashLEDS(ledSubsystem.strip0, greenFlash).run();
            ledSubsystem.strip0.solidColorRGB(0, 0, 0);
            ledSubsystem.strip0.set();
        } 
        
        return speakerSubsystem.hasNote;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.intakeMotor.set(0);
        speakerSubsystem.feedMotor.set(0);
    }
}
