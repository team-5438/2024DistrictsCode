package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
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
    public GenericEntry autoAimingShuffleBoard;
    public GenericEntry autoAimPivotEncoderShuffleBoard;

    public ColorSensorV3 colorSensor;

    public boolean isRevved;
    public RelativeEncoder topEncoder;
    public RelativeEncoder bottomEncoder;

    public SpeakerSubsystem() {
        pivotMotor = new CANSparkMax(Constants.Shooter.Speaker.pivotID, MotorType.kBrushless);
        feedMotor = new CANSparkMax(Constants.Shooter.Speaker.feedMotorID, MotorType.kBrushless);
        feedMotor.setInverted(true);

        topShootMotor = new CANSparkMax(Constants.Shooter.Speaker.topShootID, MotorType.kBrushless);
        bottomShootMotor = new CANSparkMax(Constants.Shooter.Speaker.bottomShootID, MotorType.kBrushless);
        bottomShootMotor.follow(topShootMotor, false);

        pivotEncoder = new DutyCycleEncoder(Constants.Shooter.Speaker.pivotEncoderDIOPort);
        pivotEncoder.setPositionOffset(Constants.Shooter.Speaker.pivotEncoderOffset);
        pivotPID = Constants.Shooter.Speaker.pivotPID;

        topEncoder = topShootMotor.getEncoder();
        bottomEncoder = bottomShootMotor.getEncoder();

        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        tab = Shuffleboard.getTab("SpeakerSubsystem");
        pivotEncoderShuffleBoard = tab.add("Pivot Encoder", 0.0).getEntry();
        speakerDistanceShuffleBoard = tab.add("Distance to Speaker", 0.0).getEntry();
        noteProximityShuffleBoard = tab.add("Proximity to Note", 0.0).getEntry();
        autoAimingShuffleBoard = tab.add("Auto Aiming", false).getEntry();
        autoAimPivotEncoderShuffleBoard = tab.add("Auto Aim Pivot Encoder", 0.0).getEntry();
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

        if (topEncoder.getVelocity() > Constants.Shooter.Speaker.revvedVelocity && bottomEncoder.getVelocity() > Constants.Shooter.Speaker.revvedVelocity) {
            isRevved = true;
        }
        else {
            isRevved = false;
        }
    }
}
