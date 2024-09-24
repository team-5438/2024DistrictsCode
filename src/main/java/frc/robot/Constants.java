package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.Controller;
import frc.robot.utils.GetAlliance;
import frc.robot.utils.StickDeadband;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    /* controller constants */
    public static final Controller Driver = new Controller(
            /* NOTE: this is a Xbox Controller */
            0, /* id */
            new StickDeadband(0.1, 0.1), /* left stick deadband */
            new StickDeadband(0.1, 0.1)); /* right stick deadband */

    public static final Controller Operator = new Controller(
            /* NOTE: this is a PS4 Controller */
            1, /* id */
            new StickDeadband(0.1, 0.1), /* left stick deadband */
            new StickDeadband(0.1, 0.1)); /* right stick deadband */

    public static final class DriveBase {
        public static final double maxSpeed = 4.5; /* in m/s */
        public static final double maxAcceleration = 2.5; /* in m/s */

        public static final class Auto {
            public static final PIDConstants translationPID = new PIDConstants(10.0, 0.0, 0.0);
            public static final PIDConstants rotationPID = new PIDConstants(0.1, 0.0, 0.0); /* TODO: tune */
            public static final String autoName = "bow tie";
        }
    }

    public static final class Climber {
        public static final int LClimberID = 18;
        public static final int RClimberID = 17;
    }

    public static final class Intake {
        public static final int intakeMotorID = 4;
    }

    public static final class Shooter {
        public static final class Speaker {
            public static final int pivotID = 15;
            public static final PIDController pivotPID = new PIDController(8.5, 0, 0.2);
            public static final ArmFeedforward pivotFeedforward = new ArmFeedforward(0.1, 0.01, 0.1);
            public static final int pivotEncoderDIOPort = 3;
            public static final double pivotEncoderOffset = 0.501;
            public static final double maxPivotSpeed = 0.2;
            public static final int proximitySensorDIOPort = 8;

            public static final int topShootID = 8;
            public static final int bottomShootID = 9;

            public static final double revvedVelocity = 1000; //in RPM, because of course it is
            public static final double shootingSpeed = 1;
            public static final double idleSpeed = 0.2;

            public static final int feedMotorID = 10;

            public static final double aimedTolerance = 0.008;
        }

        public static final class Amp {
            public static final int pivotID = 7;
            public static final int shooterID = 16;
            public static final double pivotEncoderOffset = 0.85;
        }
    }

    public static final class Vision {
        public static final String camera0 = "Camera_Module_v1";
        public static final double alignedTolerance = 7.5;
    }

    public static final class LEDs {
        public static final int[] redDefault = { 255, 0, 0 };
        public static final int[] blueDefault = { 0, 0, 255 };

        public static final int[] ledDefault = Robot.isRedAlliance ? redDefault : blueDefault;
    }

    /* april tags dependant on alliance */
    public static final class AprilTags {
        /* source */
        public static final long sourceL = Robot.isRedAlliance ? 10 : 2;
        public static final long sourceR = Robot.isRedAlliance ? 9 : 1;

        /* amp */
        public static final long amp = Robot.isRedAlliance ? 5 : 6;

        /* stage */
        // TODO: docs aren't clear about the positions
        // public static final long stageA = Robot.isRedAlliance ?  : ;
        // public static final long stageB = Robot.isRedAlliance ?  : ;
        // public static final long stageC = Robot.isRedAlliance ?  : ;

        /* speaker */
        public static final long speakerCentral = Robot.isRedAlliance ? 4 : 7;
        public static final long speakerOffset = Robot.isRedAlliance ? 3 : 8;
    }

    public static final class Robot {
        /* this boolean is used to determine if we should flip the path in path planner */
        public static final boolean isRedAlliance = GetAlliance.isRed();
    }
}