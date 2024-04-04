package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;

import frc.robot.Constants;

public class SpeakerSubsystem extends SubsystemBase {
    //private NetworkTable NT;
    public CANSparkMax pivotMotor;
    public CANSparkMax topShootMotor;
    public CANSparkMax bottomShootMotor;
    public CANSparkMax feedMotor;

    public DutyCycleEncoder pivotEncoder;
    public PIDController pivotPID;
    public ArmFeedforward pivotFeedforward;
    public double pivotEncoderDistance; /* Absolute value of distance use this
                                           instead of pivotEncoder.getDistance() */
    public DigitalInput proximitySensor;
    public boolean hasNote;

    public double colorSensorProximity;

    /* shuffleboard variables */
    public ShuffleboardTab tab;
    public GenericEntry pivotEncoderShuffleBoard;
    public GenericEntry speakerDistanceShuffleBoard;
    public GenericEntry noteProximityShuffleBoard;
    public GenericEntry autoAimingShuffleBoard;
    public GenericEntry autoAimPivotEncoderShuffleBoard;
    public GenericEntry speakerRevVelocityShuffleBoard;
    public GenericEntry hasNoteShuffleBoard;
    public GenericEntry readyToShootShuffleBoard;
    public GenericEntry isRevvedShuffleBoard;
    public GenericEntry autoAimedShuffleBoard;

    public ColorSensorV3 colorSensor;

    public boolean isRevved;
    public RelativeEncoder topEncoder;
    public RelativeEncoder bottomEncoder;

    public SpeakerSubsystem() {
       // NT = NetworkTableInstance.getDefault().getTable("photonvision");
        pivotMotor = new CANSparkMax(Constants.Shooter.Speaker.pivotID, MotorType.kBrushless);
        feedMotor = new CANSparkMax(Constants.Shooter.Speaker.feedMotorID, MotorType.kBrushless);
        feedMotor.setInverted(true);

        topShootMotor = new CANSparkMax(Constants.Shooter.Speaker.topShootID, MotorType.kBrushless);
        bottomShootMotor = new CANSparkMax(Constants.Shooter.Speaker.bottomShootID, MotorType.kBrushless);
        bottomShootMotor.follow(topShootMotor, false);

        pivotEncoder = new DutyCycleEncoder(Constants.Shooter.Speaker.pivotEncoderDIOPort);
        pivotEncoder.setPositionOffset(Constants.Shooter.Speaker.pivotEncoderOffset);
        pivotPID = Constants.Shooter.Speaker.pivotPID;
        pivotFeedforward = Constants.Shooter.Speaker.pivotFeedforward;

        topEncoder = topShootMotor.getEncoder();
        bottomEncoder = bottomShootMotor.getEncoder();

        colorSensor = new ColorSensorV3(I2C.Port.kMXP);
        colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate6ms);

        tab = Shuffleboard.getTab("SpeakerSubsystem");
        pivotEncoderShuffleBoard = tab.add("Pivot Encoder", 0.0).getEntry();
        speakerDistanceShuffleBoard = tab.add("Distance to Speaker", 0.0).getEntry();
        noteProximityShuffleBoard = tab.add("Proximity to Note", 0.0).getEntry();
        autoAimingShuffleBoard = tab.add("Auto Aiming", false).getEntry();
        autoAimPivotEncoderShuffleBoard = tab.add("Auto Aim Pivot Encoder", 0.0).getEntry();
        speakerRevVelocityShuffleBoard = tab.add("Rev Velocity", 0.0).getEntry();
        hasNoteShuffleBoard = tab.add("Has Note", false).getEntry();
        readyToShootShuffleBoard = tab.add("Ready To Shoot", false).getEntry();
        isRevvedShuffleBoard = tab.add("Revved", false).getEntry();
        autoAimedShuffleBoard = tab.add("Auto Aimed", false).getEntry();
    }

    @Override
    public void periodic() {
        double topEncoderVelocity;
        topEncoderVelocity = topEncoder.getVelocity();

        pivotEncoderDistance = Math.abs(pivotEncoder.getDistance());
        colorSensorProximity = colorSensor.getProximity();

        if (colorSensorProximity > 150) {
            hasNote = true;
        } else {
            hasNote = false;
        }
        hasNoteShuffleBoard.setBoolean(hasNote);

        /* add data to shuffleboard here */
        pivotEncoderShuffleBoard.setDouble(pivotEncoderDistance);
        noteProximityShuffleBoard.setDouble(colorSensorProximity);
        speakerRevVelocityShuffleBoard.setDouble(topEncoderVelocity);

        if (Math.abs(topEncoderVelocity) > Constants.Shooter.Speaker.revvedVelocity) {
            isRevved = true;
        } else {
            isRevved = false;
        }
        isRevvedShuffleBoard.setBoolean(isRevved);
    }
}