package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.SpeakerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class AutoAimSpeakerCommand extends Command {
    private PhotonSubsystem photonSubsystem;
    private SpeakerSubsystem speakerSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private LEDSubsystem ledSubsystem;
    private PhotonTrackedTarget tag;

    /* speed of speaker pivot as a percentage */
    private double pivotSpeed;
    /* distance from the speaker in inches */
    private double speakerDistance;
    /* desired angle of the shooter in radians */
    private double speakerAngle;

    private double[] distanceLookup = { 1.38,  1.57,  1.83,  2.04,  2.29,  2.55,  2.84,  3.09, 3.35,  3.58,  3.8,   4.1, 4.4, 4.7, 4.94, 5.28 };
    private double[] encoderLookup =  { 0.146, 0.143, 0.133, 0.121, 0.112, 0.104, 0.094, 0.09, 0.084, 0.081, 0.078, 0.075, 0.071, 0.064, 0.058, 0.057 };

    public AutoAimSpeakerCommand(SpeakerSubsystem speakerSubsystem, PhotonSubsystem photonSubsystem, LEDSubsystem ledSubsystem, SwerveSubsystem swerveSubsystem) {
        this.speakerSubsystem = speakerSubsystem;
        this.photonSubsystem = photonSubsystem;
        this.ledSubsystem = ledSubsystem;        
        this.swerveSubsystem = swerveSubsystem;
        
        /* override default aiming */
        addRequirements(speakerSubsystem);
    }

    @Override
    public void initialize() {
        /* show that we are auto aiming */
        speakerSubsystem.autoAimingShuffleBoard.setBoolean(true);
    }

    @Override
    public void execute() {
        speakerDistance = 0.0;
        speakerAngle = 0.0;

        tag = photonSubsystem.getTag(Constants.AprilTags.speakerCentral);
        if (swerveSubsystem.getPose().getX() < 7) {
            if (speakerSubsystem.hasNote) {
                /* if so we need to spin up our shooting wheels */
                speakerSubsystem.topShootMotor.set(Constants.Shooter.Speaker.shootingSpeed);
            } else {
                if (!DriverStation.isAutonomous()) {
                    speakerSubsystem.topShootMotor.set(Constants.Shooter.Speaker.idleSpeed);
                }
            }
        }
        if (tag == null) {
            /* return here to ensure we don't use non-existent tag info */
            return;
        } else if (speakerDistance > 7.26) {
            /* return here to ensure we don't shoot out of range */
            System.out.println("Out of acceptable shooting range");
            return;
        }
        /* calculate the distance from the bottom of the speaker to the robot */
        // speakerDistance = Units.metersToInches(Math.abs(tag.getBestCameraToTarget().getX() - 1.592));
        speakerDistance = tag.getBestCameraToTarget().getX();
        /* send distance data to shuffleboard */
		speakerSubsystem.speakerDistanceShuffleBoard.setDouble(speakerDistance);

        int lowerBoundIndex = 0;
        for (int i = distanceLookup.length - 1; i >= 0 && lowerBoundIndex == 0; i--) {
            if (distanceLookup[i] <= speakerDistance) {
                lowerBoundIndex = i;
            }
        }
        if (lowerBoundIndex >= distanceLookup.length - 2) {
            speakerAngle = encoderLookup[encoderLookup.length - 1];
        } else {
            double alpha = (speakerDistance - distanceLookup[lowerBoundIndex]) / (distanceLookup[lowerBoundIndex + 1] - distanceLookup[lowerBoundIndex]);
            speakerAngle = encoderLookup[lowerBoundIndex] + alpha * (encoderLookup[lowerBoundIndex + 1] - encoderLookup[lowerBoundIndex]);
        }

        /* calculate the correct angle to shoot into the speaker */
        // if (speakerDistance < 200) {
        //     if (speakerDistance < 36) {
        //         speakerAngle = 0.12;
        //     } else if (speakerDistance > 36 && speakerDistance < 48 ) {
        //         speakerAngle = -0.0005734 * (speakerDistance - 48) + 0.129;
        //     } else if (speakerDistance > 48 && speakerDistance < 133.914 ) {
        //         speakerAngle = (6.3 / speakerDistance );
        //     } else if (speakerDistance > 133.914 ) {
        //         speakerAngle = -0.000105 * (speakerDistance - 264) + 0.035;
        //     }
        //     speakerAngle += 0.005;
        // } else {
        //     speakerAngle = 0.12;
        // }

        /* formula version of the above */
        if (speakerDistance < 1.49) {
            speakerAngle = 0.204 * Math.pow(Math.E, -0.257 * speakerDistance);
            speakerAngle += 0.003;
        } else if (speakerDistance > 4.985) {
            speakerAngle = -0.005 * (speakerDistance - 4.94) + 0.065;
            speakerAngle -= 0.003;
        } else {
            speakerAngle = -0.00071016 * Math.pow(speakerDistance, 4) + 0.008413 *
                    Math.pow(speakerDistance, 3) - 0.02889166 *
                            Math.pow(speakerDistance, 2)
                    - 0.00144774 * speakerDistance
                    + 0.18631159;
            speakerAngle -= 0.001;
        }

        /* auto aiming angle to shuffleboard */
        speakerSubsystem.autoAimPivotEncoderShuffleBoard.setDouble(speakerAngle);

        pivotSpeed = speakerSubsystem.pivotPID.calculate(speakerSubsystem.pivotEncoderDistance, speakerAngle);
        speakerSubsystem.pivotMotor.set(pivotSpeed);

        /* check if our current angle is accurate and if we have a note */
        double aimError = Math.abs(speakerSubsystem.pivotEncoderDistance - speakerAngle);
        if (aimError <= Constants.Shooter.Speaker.aimedTolerance) {
            speakerSubsystem.autoAimedShuffleBoard.setBoolean(true);
        } else {
            speakerSubsystem.autoAimedShuffleBoard.setBoolean(false);
        }
        if (speakerSubsystem.hasNote) {
            /* if so we need to spin up our shooting wheels */
            speakerSubsystem.topShootMotor.set(Constants.Shooter.Speaker.shootingSpeed);

            /* check if shooter wheels are revved and robot is aligned with speaker */
            if (speakerSubsystem.isRevved && aimError <= Constants.Shooter.Speaker.aimedTolerance){
                /* if so, all 3 conditions are fulfilled and LEDs are green */
                speakerSubsystem.readyToShootShuffleBoard.setBoolean(true);
                ledSubsystem.setGreen();
            } else {
                /* otherwise, LEDs are default */
                speakerSubsystem.readyToShootShuffleBoard.setBoolean(false);
                ledSubsystem.setDefault();
            }
        } else {
            /* otherwise lets slow them down to a low speed */
            if (speakerSubsystem.topEncoder.getVelocity() < 140 || !speakerSubsystem.hasNote) {
                if (!DriverStation.isAutonomous()) {
                    speakerSubsystem.topShootMotor.set(Constants.Shooter.Speaker.idleSpeed);
                }
            }
            ledSubsystem.setDefault();
        }
    }

    @Override
    public boolean isFinished() {
        /* this command ends when we don't see our desired tag
         * or if the distance is too high to shoot accurately 
         * if we are not in autonomous mode
         * TEST: find what distance is actually inaccurate */
        if ((tag == null || speakerDistance > 7.62 && !DriverStation.isAutonomous()) && swerveSubsystem.getPose().getX() > 7.62) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        /* we also make sure to stop the motors to ensure nothing funny happens */
        if (tag == null || speakerDistance > 7) {
            speakerSubsystem.topShootMotor.set(0);
        }
        /* show that we are no longer auto aiming */
        speakerSubsystem.autoAimingShuffleBoard.setBoolean(false);
        speakerSubsystem.autoAimedShuffleBoard.setBoolean(false);

        ledSubsystem.setDefault();
    }
}