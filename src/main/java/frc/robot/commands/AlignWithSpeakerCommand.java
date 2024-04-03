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
        tag = photonSubsystem.getTag(Constants.AprilTags.speakerCentral);
        if (tag != null)
            rotSpeed = rotationPID.calculate(tag.getYaw(), 0.0) / 6;
        else
            return;
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(new Translation2d(0, 0), -rotSpeed, false);

      /*  // Get robot position
        Pose2d robotPose = swerveSubsystem.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        System.out.println("robotX: " + robotX);
        System.out.println("robotY: " + robotY);
        photonSubsystem.robotX.setDouble(robotX);
        photonSubsystem.robotY.setDouble(robotY);

        // Get tag position
        Optional<Pose3d> tagPose = photonSubsystem.aprilTagFieldLayout.getTagPose((int)Constants.AprilTags.speakerCentral);
        double tagX = tagPose.get().getX();
        double tagY = tagPose.get().getY();

        photonSubsystem.tagX.setDouble(tagX);
        photonSubsystem.tagY.setDouble(tagY);

        // Get the difference and then get angle using arctangent
        double differenceX = robotX - tagX;
        double differenceY = robotY - tagY;
        double setAngle = Units.radiansToDegrees(Math.atan2(differenceY, differenceX)) + 90;
        photonSubsystem.setAngle.setDouble(setAngle);

        System.out.println("setAngle : " + setAngle);

        rotSpeed = rotationPID.calculate(swerveSubsystem.getHeading().getDegrees(), setAngle) / 6;
        swerveSubsystem.drive(new Translation2d(0, 0), rotSpeed, false);
        */
    }

     @Override
     public boolean isFinished() {
         if (tag == null || tag.getYaw() < Constants.Vision.alignedTolerance) {
             return true;
         }
         return false;
     }
}