package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to 
 * call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        try {
            CameraServer.startAutomaticCapture();
        } catch(Error e) {
            System.out.println("Couldn't start the camera server");
            e.printStackTrace();
        }
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        /* constantly update robot pose using pose estimation */
        Optional<EstimatedRobotPose> pose = m_robotContainer.photonSubsystem.getEstimatedPose();
        if (pose != null) {
            m_robotContainer.swerveSubsystem.swerveDrive.addVisionMeasurement(
                pose.get().estimatedPose.toPose2d(),
                pose.get().timestampSeconds);
        }
    }

    /* This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /* This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        /* make auto aiming default in autonomous */
        m_robotContainer.speakerSubsystem.setDefaultCommand(m_robotContainer.autoAimSpeakerCommand);

        /* schedule the autonomous command */
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        } 
    }

    /* This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        /* always intake during autos to make path planning simpler */
        m_robotContainer.intakeSubsystem.intakeMotor.set(1);
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        /* make manual aiming default in teleop mode */
        m_robotContainer.speakerSubsystem.setDefaultCommand(m_robotContainer.manualAimSpeakerCommand);
    }

    /* This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        /* if we can see the right april tag we start auto aiming here */
        if (m_robotContainer.ampSubsystem.pivotEncoder.getPosition() > 0.2 && m_robotContainer.speakerSubsystem.pivotEncoderDistance < 0.082) {
            m_robotContainer.speakerSubsystem.pivotMotor.set(m_robotContainer.speakerSubsystem.pivotPID.calculate(m_robotContainer.speakerSubsystem.pivotEncoderDistance, 0.082));
        }
        if (m_robotContainer.photonSubsystem.getTag(Constants.AprilTags.speakerCentral) != null) {
            m_robotContainer.autoAimSpeakerCommand.schedule();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /* This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /* This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /* This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
