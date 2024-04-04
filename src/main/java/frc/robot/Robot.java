package frc.robot;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
        m_robotContainer.ledSubsystem.setDefault();

        addPeriodic(() -> {
            m_robotContainer.speakerSubsystem.colorSensorProximity = m_robotContainer.speakerSubsystem.colorSensor.getProximity();
        }, 0.05);
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

        m_robotContainer.swerveSubsystem.configCurrentLimits(40);
        
        /* make auto aiming default in autonomous */
        m_robotContainer.speakerSubsystem.setDefaultCommand(m_robotContainer.autoAimSpeakerCommand);

        /* schedule the autonomous command */
        if (m_autonomousCommand != null) {
            m_robotContainer.swerveSubsystem.swerveDrive.addVisionMeasurement(PathPlannerAuto.getStaringPoseFromAutoFile(Constants.DriveBase.Auto.autoName), Timer.getFPGATimestamp());
            m_autonomousCommand.schedule();
        } 
    }

    /* This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        /* always intake during autos to make path planning simpler */
        m_robotContainer.intakeSubsystem.intakeMotor.set(1);

        m_robotContainer.speakerSubsystem.topShootMotor.set(Constants.Shooter.Speaker.shootingSpeed);
        m_robotContainer.speakerSubsystem.bottomShootMotor.set(Constants.Shooter.Speaker.shootingSpeed);
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        m_robotContainer.intakeSubsystem.intakeMotor.set(0);
        m_robotContainer.swerveSubsystem.configCurrentLimits(80);
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        /* make manual aiming default in teleop mode */
        m_robotContainer.speakerSubsystem.setDefaultCommand(m_robotContainer.manualAimSpeakerCommand);
    }

    /* This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if (Math.abs(MathUtil.applyDeadband(-m_robotContainer.operator.getRightY(), Constants.Operator.rightStick.Y)) > 0.05
            || m_robotContainer.operator.getL2Axis() > 0.1) {
            if (m_robotContainer.ampPresetCommand.isScheduled()) {
                m_robotContainer.ampPresetCommand.cancel();
            }
            if (m_robotContainer.autoAimSpeakerCommand.isScheduled()) {
                m_robotContainer.autoAimSpeakerCommand.cancel();
            }
        } else if (m_robotContainer.ampSubsystem.pivotEncoder.getPosition() > 0.13) {
            /* if the amp if over a certain angle go into amp preset mode */
            if (m_robotContainer.ampPresetCommand.isScheduled()) {
                m_robotContainer.ampPresetCommand.cancel();
            }
            if (m_robotContainer.autoAimSpeakerCommand.isScheduled()) {
                m_robotContainer.autoAimSpeakerCommand.cancel();
            }
            m_robotContainer.ampPresetCommand.schedule();
        } else if (m_robotContainer.photonSubsystem.getTag(Constants.AprilTags.speakerCentral) != null || m_robotContainer.swerveSubsystem.getPose().getX() < 7) {
            /* if we can see the right april tag we start auto aiming here */
            if (m_robotContainer.ampPresetCommand.isScheduled()) {
                m_robotContainer.ampPresetCommand.cancel();
            }
            if (m_robotContainer.autoAimSpeakerCommand.isScheduled()) {
                m_robotContainer.autoAimSpeakerCommand.cancel();
            }
            m_robotContainer.autoAimSpeakerCommand.schedule();
        }

        /* constantly update robot pose using pose estimation */
        EstimatedRobotPose pose = m_robotContainer.photonSubsystem.getEstimatedPose();
        if (pose != null) {
            m_robotContainer.swerveSubsystem.swerveDrive.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(),
                    pose.timestampSeconds);
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