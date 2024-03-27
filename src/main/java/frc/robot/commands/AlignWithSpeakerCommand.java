package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignWithSpeakerCommand extends Command {
    private PhotonSubsystem photonSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private PIDController rotationPID;
    private PhotonTrackedTarget tag;

    private double rotSpeed;


    public AlignWithSpeakerCommand(PhotonSubsystem photonSubsystem, SwerveSubsystem swerveSubsystem) {
        this.photonSubsystem = photonSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        rotationPID = new PIDController(0.3, 0.0, 0.0);

        /* setup some tolerance to allow us to aim "faster" */

        /* get the tag we want to aim at */
        tag = photonSubsystem.getTag(Constants.AprilTags.speakerCentral);

        /*we're not aligned just yet, so isAligned is false */
    }

    @Override
    public void execute() {
        if (tag != null) {
            rotSpeed = rotationPID.calculate(tag.getYaw(), 0) / 6;
        } else {
            rotSpeed = 0;
        }
        swerveSubsystem.drive(new Translation2d(0, 0), rotSpeed, false);
    }

    @Override
    public boolean isFinished() {
        if (tag.getYaw() < Constants.Vision.alignedTolerance || tag == null) {
            return true;
        }
        return false;
    }
}
