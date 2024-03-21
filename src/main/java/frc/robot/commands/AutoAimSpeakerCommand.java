package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.SpeakerSubsystem;
import frc.robot.subsystems.PhotonSubsystem;

public class AutoAimSpeakerCommand extends Command {
    private PhotonSubsystem photonSubsystem;
    private SpeakerSubsystem speakerSubsystem;
    private PhotonTrackedTarget tag;

    /* speed of speaker pivot as a percentage */
    private double pivotSpeed;
    /* distance from the speaker in inches */
    private double speakerDistance;
    /* desired angle of the shooter in radians */
    private double speakerAngle;

    public AutoAimSpeakerCommand(SpeakerSubsystem speakerSubsystem, PhotonSubsystem photonSubsystem) {
        this.speakerSubsystem = speakerSubsystem;
        this.photonSubsystem = photonSubsystem;
        
        /* override default aiming */
        addRequirements(speakerSubsystem);
    }

    @Override
    public void initialize() {
        /* on startup we make sure the motors spin at a low speed to make
         * revving up faster 
         * TEST: if this actually changes how fast we actually rev up */
        speakerSubsystem.topShootMotor.set(Constants.Shooter.Speaker.idleSpeed);
        /* show that we are auto aiming */
        speakerSubsystem.autoAiming.setBoolean(true);
    }

    @Override
    public void execute() {
        speakerDistance = 0.0;
        speakerAngle = 0.0;

        tag = photonSubsystem.getTag(Constants.AprilTags.speakerCentral);
        if (tag == null) {
            /* return here to ensure we don't use non-existent tag info */
            return;
        }
        /* calculate the distance from the bottom of the speaker to the robot */
        speakerDistance = Units.metersToInches(Math.abs(tag.getBestCameraToTarget().getX() - 1.592));
        /* send distance data to shuffleboard */
		speakerSubsystem.speakerDistanceShuffleBoard.setDouble(speakerDistance);

        /* calculate the correct angle to shoot into the speaker */
        if (speakerDistance < 200) {
            if (speakerDistance < 36) {
                speakerAngle = 0.12;
            } else if (speakerDistance > 36 && speakerDistance < 48 ) {
                speakerAngle = -0.0005734 * (speakerDistance - 48) + 0.129;
            } else if (speakerDistance > 48 && speakerDistance < 133.914 ) {
                speakerAngle = (6.3 / speakerDistance );
            } else if (speakerDistance > 133.914 ) {
                speakerAngle = -0.000105 * (speakerDistance - 264) + 0.035;
            }
            speakerAngle += 0.005;
        } else {
            speakerAngle = 0.12;
        }

        pivotSpeed = speakerSubsystem.pivotPID.calculate(speakerAngle);
        speakerSubsystem.pivotMotor.set(pivotSpeed);

        /* check if our current angle is accurate and if we have a note */
        if (Math.abs(speakerSubsystem.pivotEncoder.getDistance() - speakerAngle) <= 0.01
                && speakerSubsystem.hasNote) {
            /* if so we need to spin up our shooting wheels */
            speakerSubsystem.topShootMotor.set(Constants.Shooter.Speaker.shootingSpeed);
        } else {
            /* otherwise lets slow them down to a low speed */
            speakerSubsystem.topShootMotor.set(Constants.Shooter.Speaker.idleSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        /* this command ends when we don't see our desired tag
         * or if the distance is too high to shoot accurately 
         * if we are not in autonomous mode
         * TEST: find what distance is actually inaccurate */
        if (tag == null || speakerDistance > 200 && !DriverStation.isAutonomous()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        /* we also make sure to stop the motors to ensure nothing funny happens */
        speakerSubsystem.topShootMotor.set(0);
        /* show that we are no longer auto aiming */
        speakerSubsystem.autoAiming.setBoolean(false);
    }
}
