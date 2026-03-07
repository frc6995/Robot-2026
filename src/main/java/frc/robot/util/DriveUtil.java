package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveUtil {
    public static Rotation2d getAngleTowards(Pose2d a, Pose2d b) {
        return getAngleTowards(a.getTranslation(), b.getTranslation());
    }

    public static Rotation2d getAngleTowards(Translation2d a, Translation2d b) {
        return a.minus(b).getAngle();
    }
}
