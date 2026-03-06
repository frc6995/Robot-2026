package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.ChoreoVars;
import static frc.robot.util.AllianceFlipUtil.flipped;

public class POI {
    // ============= POSES =============

    // Center line intake start poses
    public static final Supplier<Pose2d> BALLL1 = flipped(ChoreoVars.Poses.BALLL1);
    public static final Supplier<Pose2d> BALLR1 = flipped(ChoreoVars.Poses.BALLR1);
    public static final Supplier<Pose2d> BALLL2 = flipped(ChoreoVars.Poses.BALLL2);
    public static final Supplier<Pose2d> BALLL3 = flipped(ChoreoVars.Poses.BALLL3);

    // Center line intake stop poses
    public static final Supplier<Pose2d> STOPL1 = flipped(ChoreoVars.Poses.STOPL1);
    public static final Supplier<Pose2d> STOPL2 = flipped(ChoreoVars.Poses.STOPL2);

    // Auto start poses
    public static final Supplier<Pose2d> TRL1 = flipped(ChoreoVars.Poses.TRL1);
    public static final Supplier<Pose2d> TRR1 = flipped(ChoreoVars.Poses.TRR1);

    // Center line help poses
    public static final Supplier<Pose2d> HELPL1 = flipped(ChoreoVars.Poses.HELPL1);
    public static final Supplier<Pose2d> HELPR1 = flipped(ChoreoVars.Poses.HELPR1);
    public static final Supplier<Pose2d> HELPL2 = flipped(ChoreoVars.Poses.HELPL2);

    // Climb poses
    public static final Supplier<Pose2d> CL1 = flipped(ChoreoVars.Poses.CL1);

    // Depot poses
    public static final Supplier<Pose2d> DEPOT_START = flipped(ChoreoVars.Poses.DEPOTSTART);
    public static final Supplier<Pose2d> DEPOT_END = flipped(ChoreoVars.Poses.DEPOTEND);
    public static final Supplier<Pose2d> DEPOT_HELP = flipped(ChoreoVars.Poses.HELPD1);

    // Other poses
    public static final Supplier<Pose2d> STA1 = flipped(ChoreoVars.Poses.STA1);
    public static final Supplier<Pose2d> HUB1 = flipped(new Pose2d(4.625, 4.05, new Rotation2d()));

    // ============= ROTATIONS =============

    // Center line intake pose rotations
    public static final Supplier<Rotation2d> BALLL2Entry = flipped(new Rotation2d(Degrees.of(-35)));
    public static final Supplier<Rotation2d> BALLL2CloseEntry = flipped(new Rotation2d(Degrees.of(0)));

    // Auto start pose rotations
    public static final Supplier<Rotation2d> TRL1Entry = flipped(new Rotation2d(Degrees.of(180)));
    public static final Supplier<Rotation2d> TRR1Entry = flipped(new Rotation2d(Degrees.of(0)));

    // Autoalign help pose rotations
    public static final Supplier<Rotation2d> HELPL1Entry = flipped(new Rotation2d(Degrees.of(0)));
    public static final Supplier<Rotation2d> HELPR1Entry = flipped(new Rotation2d(Degrees.of(-30)));
    public static final Supplier<Rotation2d> HELPL2Entry = flipped(new Rotation2d(Degrees.of(160)));
    public static final Supplier<Rotation2d> HELPL2CloseEntry = flipped(new Rotation2d(Degrees.of(140)));

    // Climb pose rotations
    // Other rotations
    public static final Supplier<Rotation2d> testEntry = flipped(new Rotation2d(Degrees.of(-90)));
    public static final Supplier<Rotation2d> depotStartEntry = flipped(new Rotation2d(Degrees.of(200)));

    // Passing zones
    public static final Translation2d centerZoneCorner1 = new Translation2d(Meters.of(4.7), Meters.of(4.7));
    public static final Translation2d centerZoneCorner2 = new Translation2d(Meters.of(11.7), Meters.of(3.3));
    public static final Supplier<Rectangle2d> topAllianceZone = ()-> new Rectangle2d(
            flipped(new Translation2d(Meters.of(0), Meters.of(0))).get(), 
            flipped(new Translation2d(Meters.of(0), Meters.of(0))).get());
    public static final Supplier<Rectangle2d> bottomAllianceZone = ()-> new Rectangle2d(
            flipped(new Translation2d(Meters.of(0), Meters.of(0))).get(), 
            flipped(new Translation2d(Meters.of(0), Meters.of(0))).get());
    public static final Supplier<Rectangle2d> topZone = ()-> new Rectangle2d(
            flipped(new Translation2d(Meters.of(0), Meters.of(0))).get(), 
            flipped(new Translation2d(Meters.of(0), Meters.of(0))).get());
    public static final Supplier<Rectangle2d> centerZone = () -> flippedRectangle(centerZoneCorner1, centerZoneCorner2);
    public static final Supplier<Rectangle2d> bottomZone = ()-> new Rectangle2d(
            flipped(new Translation2d(Meters.of(4.7), Meters.of(0.5))).get(), 
            flipped(new Translation2d(Meters.of(0), Meters.of(0))).get());

    // ============= TRANSLATIONS =============
    // ============= DISTANCES =============
    public static final Distance kOriginToTrenchBlue = Meters.of(4.6);
    public static final Distance kOriginToTrenchRed = Meters.of(11.9);

    private static Rectangle2d flippedRectangle(Translation2d corner1, Translation2d corner2) {
        return new Rectangle2d(flipped(corner1).get(), flipped(corner2).get());
    }
}