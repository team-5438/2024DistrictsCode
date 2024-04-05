package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignWithSpeakerCommand;
import frc.robot.commands.AmpPresetCommand;
import frc.robot.commands.AmpShootCommand;
import frc.robot.commands.AutoAimSpeakerCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualAimAmpCommand;
import frc.robot.commands.ManualAimSpeakerCommand;
import frc.robot.commands.RevShooterWheelsCommand;
import frc.robot.commands.SetSpeakerPositionCommand;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SpeakerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    public final SpeakerSubsystem speakerSubsystem;
    public final AmpSubsystem ampSubsystem;
    public final PhotonSubsystem photonSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final ClimberSubsystem climberSubsystem;
    public final LEDSubsystem ledSubsystem;

    public final ManualAimSpeakerCommand manualAimSpeakerCommand;
    public final AutoAimSpeakerCommand autoAimSpeakerCommand;
    public final ManualAimAmpCommand manualAimAmpCommand;
    public final AmpPresetCommand  ampPresetCommand;

    public final XboxController driver = new XboxController(Constants.Driver.id);
    public final PS4Controller operator = new PS4Controller(Constants.Operator.id);

    /* The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        // The robot's subsystems and commands are defined here...
        /* subsystems */
        speakerSubsystem = new SpeakerSubsystem();
        System.out.println("Speaker Subsystem");
        ampSubsystem = new AmpSubsystem();
        System.out.println("Amp Subsystem");
        photonSubsystem = new PhotonSubsystem();
        System.out.println("Photon Subsystem");
        intakeSubsystem = new IntakeSubsystem();
        System.out.println("Intake Subsystem");
        climberSubsystem = new ClimberSubsystem();
        System.out.println("Climber Subsystem");
        ledSubsystem = new LEDSubsystem();
        System.out.println("Led Subsystem");

        /* commands */
        manualAimSpeakerCommand = new ManualAimSpeakerCommand(speakerSubsystem, operator);
        autoAimSpeakerCommand = new AutoAimSpeakerCommand(speakerSubsystem, photonSubsystem, ledSubsystem, swerveSubsystem, operator);
        manualAimAmpCommand = new ManualAimAmpCommand(ampSubsystem, operator);
        ampPresetCommand = new AmpPresetCommand(speakerSubsystem, ampSubsystem);

        /* set default commands (NOTE: some of these commands may have binds inside of them) */
        ampSubsystem.setDefaultCommand(manualAimAmpCommand);

        // The robot's bindings are defined here...
        configureBindings();

        // robots named commands
        namedCommands();
    }

    private void configureBindings() {
        /* translation controls for the robot */
        DoubleSupplier speedMod = () -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) == 1 ? 1.5 : 1;
        /* NOTE: the division is used to reduce the speed of the robot when the left trigger is held */
        DoubleSupplier translationX = () -> -MathUtil.applyDeadband(driver.getLeftY(), Constants.Driver.leftStick.Y) / speedMod.getAsDouble();
        DoubleSupplier translationY = () -> -MathUtil.applyDeadband(driver.getLeftX(), Constants.Driver.leftStick.X) / speedMod.getAsDouble();

        /* rotation controls for the robot */
        DoubleSupplier angularRotationX = () -> -MathUtil.applyDeadband(driver.getRawAxis(4), Constants.Driver.rightStick.X) / speedMod.getAsDouble();

        /* Boolean supplier which determines if the robot is field or robot oriented */
        BooleanSupplier robotOriented = () -> !driver.getLeftBumper();
        BooleanSupplier reverseRobotOriented = () -> !driver.getRightBumper();

        /* define and register the swerve controls */
        Command driverControls = swerveSubsystem.driveCommand(translationX, translationY, angularRotationX, robotOriented, reverseRobotOriented);
        swerveSubsystem.setDefaultCommand(driverControls);

        /* configure binds */
        /* driver binds */
        new JoystickButton(driver, XboxController.Button.kY.value).onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
        new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new AlignWithSpeakerCommand(photonSubsystem, swerveSubsystem));
        // new JoystickButton(driver, XboxController.Button.kX.value).onTrue(new DriveToAmpCommand(swerveSubsystem, driver));

        /* operator binds */
        /* intake */
        new JoystickButton(operator, PS4Controller.Button.kSquare.value).onTrue(new SequentialCommandGroup(
            new IntakeCommand(intakeSubsystem, speakerSubsystem, ledSubsystem).withTimeout(10),
            new InstantCommand(() -> speakerSubsystem.feedMotor.set(-0.1)),
            new WaitCommand(0.1),
            new InstantCommand(() -> speakerSubsystem.feedMotor.set(0)),
            new InstantCommand(() -> ledSubsystem.setDefault())
        ));

        /* shoot note out of speaker shooter */
        new JoystickButton(operator, PS4Controller.Button.kR2.value).whileTrue( new FeedCommand(speakerSubsystem, intakeSubsystem, 1));

        /* rev up speaker shooter wheels manually */
        new JoystickButton(operator, PS4Controller.Button.kL2.value).whileTrue(new RevShooterWheelsCommand(speakerSubsystem, Constants.Shooter.Speaker.shootingSpeed));
        /* and back them up just incase */
        new JoystickButton(operator, PS4Controller.Button.kL1.value).whileTrue(new ParallelCommandGroup(
            new RevShooterWheelsCommand(speakerSubsystem, -0.3),
            new FeedCommand(speakerSubsystem, intakeSubsystem, -0.1)
        ));

        /* shoot note out of amp shooter */
        new JoystickButton(operator, PS4Controller.Button.kTriangle.value).whileTrue(new AmpShootCommand(ampSubsystem, speakerSubsystem, 1));
        new JoystickButton(operator, PS4Controller.Button.kCircle.value).whileTrue(new AmpShootCommand(ampSubsystem, speakerSubsystem, -1));

        /* bring climbers up and down */
        new JoystickButton(operator, PS4Controller.Button.kOptions.value).whileTrue(new ClimbCommand(climberSubsystem, 0.94));
        new JoystickButton(operator, PS4Controller.Button.kShare.value).whileTrue(new ClimbCommand(climberSubsystem, -0.94));

        /* Source preset */
        new JoystickButton(operator, PS4Controller.Button.kCross.value).whileTrue(new SetSpeakerPositionCommand(speakerSubsystem, 0.1176));
    }

    private void namedCommands() {
        NamedCommands.registerCommand("Shoot", new FeedCommand(speakerSubsystem, intakeSubsystem, 1).withTimeout(1));
        NamedCommands.registerCommand("Align with Speaker", new AlignWithSpeakerCommand(photonSubsystem, swerveSubsystem));
        NamedCommands.registerCommand("Is Revved", new WaitUntilCommand(() -> speakerSubsystem.isRevved));
        // NamedCommands.registerCommand("Aim Speaker", new SetSpeakerPositionCommand(speakerSubsystem, 0.0));
    }

    public Command getAutonomousCommand() {
        System.out.println("Running autonomous...");

        // Follow Path
        return swerveSubsystem.followAuto(Constants.DriveBase.Auto.autoName);
    }
}