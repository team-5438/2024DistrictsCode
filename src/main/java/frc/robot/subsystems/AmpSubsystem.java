package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class AmpSubsystem extends SubsystemBase {
    public CANSparkMax shootMotor;
    public CANSparkMax pivotMotor;
    
    public SparkAbsoluteEncoder pivotEncoder;
    public PIDController pivotPIDController;

    /* shuffleboard variables */
    public ShuffleboardTab tab;
    public GenericEntry pivotEncoderShuffleBoard;

    public AmpSubsystem() {
        shootMotor = new CANSparkMax(Constants.Shooter.Amp.shooterID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(Constants.Shooter.Amp.pivotID, MotorType.kBrushless);

        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotEncoder.setZeroOffset(Constants.Shooter.Amp.pivotEncoderOffset);

        tab = Shuffleboard.getTab("Amp Subsystem");
        pivotEncoderShuffleBoard = tab.add("Pivot Encoder", 0.0).getEntry();
    }

    @Override
    public void periodic() {
        /* add data to shuffleboard here */
        pivotEncoderShuffleBoard.setDouble(pivotEncoder.getPosition());
    }
}