package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    public CANSparkMax LClimber;
    public CANSparkMax RClimber;

    public ClimberSubsystem() {
        /* TEST: lets make sure that setting the motors to 1 is up and -1 is down */
        LClimber = new CANSparkMax(Constants.Climber.LClimberID, MotorType.kBrushless);
        RClimber = new CANSparkMax(Constants.Climber.RClimberID, MotorType.kBrushless);

        LClimber.setInverted(true);
    }
}
