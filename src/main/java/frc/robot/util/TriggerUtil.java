package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class TriggerUtil {
    public static BooleanSupplier isWithinRadius(Supplier<Translation2d> targetLocation, Supplier<Pose2d> robotPose, Distance tolerance) {
        return () -> targetLocation.get().getDistance(robotPose.get().getTranslation()) < tolerance.in(Meters);
    }

    public static BooleanSupplier isWithinZone(Supplier<Translation2d> lowerLeftCorner, Supplier<Translation2d> upperRightCorner, Supplier<Pose2d> robotPose) {
        return () -> lowerLeftCorner.get().getX() < robotPose.get().getX() && lowerLeftCorner.get().getY() < robotPose.get().getY() &&
            upperRightCorner.get().getX() > robotPose.get().getX() && upperRightCorner.get().getY() > robotPose.get().getY();
    }

    public static BooleanSupplier negate(BooleanSupplier booleanSupplier) {
        return () -> !booleanSupplier.getAsBoolean();
    }

    public static BooleanSupplier and(BooleanSupplier supplierOne, BooleanSupplier supplierTwo) {
        return () -> supplierOne.getAsBoolean() && supplierTwo.getAsBoolean();
    }

    public static BooleanSupplier or(BooleanSupplier supplierOne, BooleanSupplier supplierTwo) {
        return () -> supplierOne.getAsBoolean() || supplierTwo.getAsBoolean();
    }
}
