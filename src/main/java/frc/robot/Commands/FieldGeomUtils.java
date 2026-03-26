package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldGeomUtils {

    public static final Pose2d RED_HUB = new Pose2d(11.907, 4.040, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_HUB = new Pose2d(4.633, 4.040, Rotation2d.fromDegrees(180));

    /**
     * Helper to get the target hub based on current alliance.
     */

    private static Pose2d getTargetHub() {
        var alliance = DriverStation.getAlliance();
        SmartDashboard.getString("Alliance", alliance.toString());

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return RED_HUB;
        }
        return BLUE_HUB;
    }

    /**
     * Calculates the angle to point the BACK of the robot at the hub.
     */
    public static Rotation2d getAngleToHub(Pose2d robotPose) {
        Translation2d relativeTranslation = getTargetHub().getTranslation().minus(robotPose.getTranslation());
        // .getAngle() is the vector from robot to hub. Adding 180 flips it to face the rear.
        var alliance = DriverStation.getAlliance();

        if (alliance.get() == DriverStation.Alliance.Red)
        {
        return relativeTranslation.getAngle().plus(Rotation2d.fromDegrees(180));
        }
        else{
        return relativeTranslation.getAngle().plus(Rotation2d.fromDegrees(0));
        }
    }

    /**
     * Calculates the straight-line distance to the target hub in meters.
     */
    public static double getDistanceToHub(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(getTargetHub().getTranslation());
    }
}