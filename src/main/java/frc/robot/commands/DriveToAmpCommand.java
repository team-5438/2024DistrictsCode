package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToAmpCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private XboxController driver;
    private Pose2d endPose;

    public DriveToAmpCommand(SwerveSubsystem swerveSubsystem, XboxController driver) {
        this.swerveSubsystem = swerveSubsystem;
        this.driver = driver;

        addRequirements(swerveSubsystem);
    } 

    @Override
    public void initialize() {
        /* get the correct pose based on the alliance we are on */
        if (Constants.Robot.isRedAlliance) {
            endPose = new Pose2d(14.72, 7.69, new Rotation2d(Units.degreesToRadians(90)));
        } else {
            endPose = new Pose2d(1.84, 7.69, new Rotation2d(Units.degreesToRadians(90)));
        }

        /* start driving to the correct pose */
        swerveSubsystem.driveToPose(endPose).schedule();
    }

    @Override
    public boolean isFinished() {
        /* if the user tries to drive or if the robot has gotten to the correct
         * pose end the command */
        if (driver.getLeftX() > Constants.Driver.leftStick.X ||
            driver.getLeftY() > Constants.Driver.leftStick.Y ||
            driver.getRightX() > Constants.Driver.rightStick.X ||
            swerveSubsystem.getPose() == endPose) {
            return true;
        }
        return false;
    }
}
