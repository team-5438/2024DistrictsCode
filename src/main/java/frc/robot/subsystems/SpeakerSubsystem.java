package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;

import frc.robot.Constants;

public class SpeakerSubsystem extends SubsystemBase {
    public CANSparkMax pivotMotor;
    public CANSparkMax topShootMotor;
    public CANSparkMax bottomShootMotor;
    public CANSparkMax feedMotor;

    public DutyCycleEncoder pivotEncoder;
    public PIDController pivotPID;
    public double pivotEncoderDistance; /* Absolute value of distance use this
                                           instead of pivotEncoder.getDistance() */
    public boolean hasNote;

    /* shuffleboard variables */
    public ShuffleboardTab tab;
    public GenericEntry pivotEncoderShuffleBoard;
    public GenericEntry speakerDistanceShuffleBoard;
    public GenericEntry noteProximityShuffleBoard;
    public GenericEntry autoAiming;

    public ColorSensorV3 colorSensor;

    public SpeakerSubsystem() {
        pivotMotor = new CANSparkMax(Constants.Shooter.Speaker.pivotID, MotorType.kBrushless);
        feedMotor = new CANSparkMax(Constants.Shooter.Speaker.feedMotorID, MotorType.kBrushless);

        topShootMotor = new CANSparkMax(Constants.Shooter.Speaker.topShootID, MotorType.kBrushless);
        bottomShootMotor = new CANSparkMax(Constants.Shooter.Speaker.bottomShootID, MotorType.kBrushless);
        bottomShootMotor.follow(topShootMotor, true);

        pivotEncoder = new DutyCycleEncoder(Constants.Shooter.Speaker.pivotEncoderDIOPort);
        pivotEncoder.setPositionOffset(Constants.Shooter.Speaker.pivotEncoderOffset);
        pivotPID = Constants.Shooter.Speaker.pivotPID;

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        tab = Shuffleboard.getTab("SpeakerSubsystem");
        pivotEncoderShuffleBoard = tab.add("Pivot Encoder", 0.0).getEntry();
        speakerDistanceShuffleBoard = tab.add("Distance to Speaker", 0.0).getEntry();
        noteProximityShuffleBoard = tab.add("Proximity to Note", 0.0).getEntry();
        autoAiming = tab.add("Auto Aiming", false).getEntry();
    }

    @Override
    public void periodic() {
        pivotEncoderDistance = Math.abs(pivotEncoder.getDistance());

        if (colorSensor.getProximity() > 150) {
            hasNote = true;
        } else {
            hasNote = false;
        }

        /* add data to shuffleboard here */
        pivotEncoderShuffleBoard.setDouble(pivotEncoderDistance);
        noteProximityShuffleBoard.setDouble(colorSensor.getProximity());
    }
}
