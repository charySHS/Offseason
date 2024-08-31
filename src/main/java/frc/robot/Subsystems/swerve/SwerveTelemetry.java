package frc.robot.Subsystems.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import pabeles.concurrency.IntOperatorTask;

import org.littletonrobotics.junction.Logger;

public class SwerveTelemetry
{
    private final double MaxSpeed;

    public SwerveTelemetry(double maxSpeed)
    {
        MaxSpeed = maxSpeed;
    }

    // what to publish over networktables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // pose
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    // robot speeds
    private final NetworkTable driveStats = inst.getTable("Drive");
    private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
    private final DoublePublisher odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();

    private Pose2d lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();

    // mechanism to represent module states
    private final Mechanism2d[] moduleMechanisms =
        new Mechanism2d[]
            {
                new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
            };

    // Direction and length changing speed representation
    private final MechanismLigament2d[] moduleSpeeds =
        new MechanismLigament2d[]
            {
                moduleMechanisms[0]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
                moduleMechanisms[1]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
                moduleMechanisms[2]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 5)),
                moduleMechanisms[3]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
            };

    // Direction changing and length constant for module direction
    private final MechanismLigament2d[] moduleDirections =
        new MechanismLigament2d[]
            {
                moduleMechanisms[0]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                moduleMechanisms[1]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                moduleMechanisms[2]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                moduleMechanisms[3]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            };

    // telemeterize swerve states to smartdashboard
    public void telemeterize(SwerveDriveState state)
    {
        // Telemeterize pose
        Pose2d pose = state.Pose;
        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees() });

        // telemeterize general speeds
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(lastPose).getTranslation();
        lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        speed.set(velocities.getNorm());
        velocityX.set(velocities.getX());
        velocityY.set(velocities.getY());
        odomPeriod.set(state.OdometryPeriod);

        // telemeterize module states
        for (int i = 0; i < 4; ++i)
        {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

            Logger.recordOutput("Module " + i, moduleMechanisms[i]);
        }

    }
}
