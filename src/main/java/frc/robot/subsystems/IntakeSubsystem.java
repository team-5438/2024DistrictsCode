package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.utils.Motor;
import frc.robot.utils.Motor.MotorBrand;

public class IntakeSubsystem extends SubsystemBase {
    public Motor intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new Motor(Constants.Intake.intakeMotorID, MotorBrand.Talon);
    }
}