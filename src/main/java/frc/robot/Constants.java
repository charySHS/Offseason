package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.pathplanner.lib.util.PIDConstants;

import friarLib2.utility.PIDParameters;

public final class Constants {

    public enum RioCanBusIDs
    {
        RoboRIO,
        PDH,
        FeederMotor
    }

    public enum CANivoreBusIDs
    {
        ArmLeft,
        ArmRight,
        IntakePivot,
        DriveBL,
        DriveBR,
        DriveFL,
        DriveFR,
        SteerBL,
        SteerBR,
        SteerFL,
        SteerFR,
        CANCoderBL,
        CANCoderBR,
        CANCoderFL,
        CANCoderFR,
        Pigeon,
        IntakeMotor;

        static public final String BusName = "CANivore";

        public int GetID()
        {
            return ordinal() + 1;
        }
    }

    /* FeatureFlags used in Robot.java */
    public static final boolean EnableAdvKit = true;
    public static final boolean EnableMonologue = false;
    public static final boolean DisableSubsystemsOnDisableInit = true;

    // No sir-ie
    public static final boolean OverrideBrownOutVoltage = true;
    public static final double OverridenBrownOutVoltage = 5.6;

    // Drive configs
    public static class Drive {
        /********** CAN ID's **********/
        //        public static final SwerveCANIDs FRONT_LEFT_MODULE_IDS = new SwerveCANIDs(32, 36, 61);
        //        public static final SwerveCANIDs FRONT_RIGHT_MODULE_IDS = new SwerveCANIDs(33, 37, 62);
        //        public static final SwerveCANIDs BACK_LEFT_MODULE_IDS = new SwerveCANIDs(34, 39, 60);
        //        public static final SwerveCANIDs BACK_RIGHT_MODULE_IDS = new SwerveCANIDs(2, 41, 59);
        //
        //        /********** Module Translations **********/
        public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(0.34671, 0.23241);
        public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(0.34671, -0.23241);
        public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(-0.34671, 0.23241);
        public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(-0.34671, -0.23241);

        /********** Autonomous Motion Envelope **********/
        public static final double MaxAutonSpeed = 2; // Meters/second
        public static final double MaxAutonAcceleration = 2.5; // Meters/second squared
        public static final double MaxAutonAngularSpeed = 400; // Degrees/second
        public static final double MaxAutonAngularAcceleration = 200; // Degrees/second squared

        /********** Holonomic Controller Gains **********/
        public static final PIDConstants HolonomicControllerPIDXYConstraints = new PIDConstants(4, .75, 0.5);
        public static final PIDConstants HolonomicControllerPIDRotationalConstraints = new PIDConstants(5, 0, 0);
        public static final PIDController HolonomicControllerPIDX = new PIDController(HolonomicControllerPIDXYConstraints.kP, HolonomicControllerPIDXYConstraints.kI, HolonomicControllerPIDXYConstraints.kD);
        public static final PIDController HolonomicControllerPIDY = new PIDController(HolonomicControllerPIDXYConstraints.kP, HolonomicControllerPIDXYConstraints.kI, HolonomicControllerPIDXYConstraints.kD);
        public static final ProfiledPIDController HolonomicControllerPIDTheta = new ProfiledPIDController(HolonomicControllerPIDRotationalConstraints.kP, HolonomicControllerPIDRotationalConstraints.kI, HolonomicControllerPIDRotationalConstraints.kD, new TrapezoidProfile.Constraints(MaxAutonAngularSpeed, MaxAutonAngularAcceleration));

        /******** PID Gains ********/
        public static final PIDController VisionAimPID = new PIDController(0.15, 0.1, 0);

        /********** Teleop Control Adjustment **********/
        public static final double MaxTeleopSpeed = 6; // Meters/second
        public static final double MaxTeleopRotationalSpeed = Math.toRadians(700); // Radians/second
        public static final double MaxTeleopAcceleration = 10; // Maters/second squared
        public static final double MaxTeleopDeceleration = 11;
    }

    public class Intake {
        /********** CAN ID's **********/
        public static final int IntakeMotorID = 23;
        public static final int PivotMotorID = 10;

        /********** Amp Scoring **********/
        public static final double IntakeMotorAmpPower = 1;

        /********** Intake  **********/
        public static final double IntakeMotorIntakePower = 1;

        /********** Trap Scoring **********/
        public static final double IntakeMotorTrapPower = 1;

        /******** Misc ********/
        public static final double TargetThreshold = 10;

        /******** Pivot PID ********/
        public static final PIDParameters PivotMotorPID = new PIDParameters(0, "Pivot Motor PID", 0.02, 0.0, 0.25, 0.0, 50); //everything in this is a placeholder

    }

    public class Indexer {
        /********** CAN ID's **********/
        public static final int IndexerMotorID = 100;

        /******** Indexer PID ********/
        public static final PIDParameters IndexerMotorPID = new PIDParameters(0, "Pivot Motor PID", 0.02, 0.0, 0.25, 0.0, 50); //everything in this is a placeholder

        /********** Indexer Motor Power **********/
        public static final double IndexerMotorPower = 1;
    }

    public class Arm {

        public static final TrapezoidProfile.Constraints ArmMotionConstraint =
            new TrapezoidProfile.Constraints(1.0, 2.0);

        public static final double ArmZeroCosineOffset = 1.342;
    }

    public class Shooter {
        /********** CAN ID's **********/
        public static final int ShooterMotorID = 98;
        public static final int Shooter2MotorID = 97;
    }

    // Logging
    public static final int LogLinesBeforeFlush = 100;
    public static final boolean MonologueFileOnly = false;
    public static final boolean MonologueLazyLogging = false;

    public static final class FeatureFlags {
        // subsystems

        public static final boolean EasterEggEnabled = false;

        public static final boolean IntakeEnabled = true;

        public static final boolean ShooterEnabled = true;

        public static final boolean SwerveEnabled = true;

        public static final boolean PivotIntakeEnabled = true;

        public static final boolean ClimbEnabled = true;

        // logging
        public static final boolean TuningMode = false;
        public static final boolean DebugEnabled = false;
        public static final boolean DebugCommandEnabled = false;
        public static final boolean RobotVizEnabled = !Robot.isReal();

        // features
        public static final boolean AutoAlignEnabled = false;
        public static final boolean IntakeAutoScoreDistanceSensorOffset = false;
        public static final boolean ShuffleboardLayoutEnabled = true;
        public static final boolean UsePrefs = true;

        public static final boolean PitRoutineEnabled = false;

        public static final boolean CANTestEnabled = false;
        public static boolean kPivotShooterEnabled = true;
    }

    public static final class ShuffleboardConstants {
        public static final String DriverTabName = "Driver";
        public static final String OperatorTabName = "Operator";
        public static final String IntakeLayoutName = "Intake";
        public static final String SwerveLayoutName = "Swerve";
        public static final String ArmLayoutName = "Arm";
        public static final String ShooterLayoutName = "Shooter";
    }
}
