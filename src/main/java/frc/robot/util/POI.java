package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import choreo.Choreo;
import choreo.util.ChoreoAllianceFlipUtil;
import choreo.util.FieldSize;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.ChoreoVars;
import static frc.robot.util.AllianceFlipUtil.flipped;

public class POI {
    // ============= POSES =============

    // Center line intake start poses
    public static final Supplier<Pose2d> BALLL1 = flipped(ChoreoVars.Poses.BALLL1);
    public static final Supplier<Pose2d> BALLL2 = flipped(ChoreoVars.Poses.BALLL2);
    public static final Supplier<Pose2d> BALLL3 = flipped(ChoreoVars.Poses.BALLL3);
    public static final Supplier<Pose2d> BALLL4 = flipped(ChoreoVars.Poses.BALLL4);
    public static final Supplier<Pose2d> BALLL5 = flipped(ChoreoVars.Poses.BALLL5);

    public static final Supplier<Pose2d> BALLR1 = flipped(ChoreoVars.Poses.BALLR1);
    public static final Supplier<Pose2d> BALLR2 = flipped(ChoreoVars.Poses.BALLR2);
    public static final Supplier<Pose2d> BALLR3 = flipped(ChoreoVars.Poses.BALLR3);

    public static final Supplier<Pose2d> BALLR5 = flipped(ChoreoVars.Poses.BALLR5);

    // Center line intake stop poses
    public static final Supplier<Pose2d> STOPL1 = flipped(ChoreoVars.Poses.STOPL1);
    public static final Supplier<Pose2d> STOPL2 = flipped(ChoreoVars.Poses.STOPL2);
    public static final Supplier<Pose2d> STOPL3 = flipped(ChoreoVars.Poses.STOPL3);
    public static final Supplier<Pose2d> STOPL4 = flipped(ChoreoVars.Poses.STOPL4);

    public static final Supplier<Pose2d> STOPR1 = flipped(ChoreoVars.Poses.STOPR1);
    public static final Supplier<Pose2d> STOPR2 = flipped(ChoreoVars.Poses.STOPR2);
    public static final Supplier<Pose2d> STOPR3 = flipped(ChoreoVars.Poses.STOPR3);
    public static final Supplier<Pose2d> STOPR4 = flipped(ChoreoVars.Poses.STOPR4);

    // Auto start poses
    public static final Supplier<Pose2d> TRL1 = flipped(ChoreoVars.Poses.TRL1);
    public static final Supplier<Pose2d> TRL2 = flipped(ChoreoVars.Poses.TRL2);
    public static final Supplier<Pose2d> TRL3 = flipped(ChoreoVars.Poses.TRL3);

    public static final Supplier<Pose2d> TRR1 = flipped(ChoreoVars.Poses.TRR1);
    public static final Supplier<Pose2d> TRR2 = flipped(ChoreoVars.Poses.TRR2);
    public static final Supplier<Pose2d> TRR3 = flipped(ChoreoVars.Poses.TRR3);

    // Center line help poses
    public static final Supplier<Pose2d> HELPL1 = flipped(ChoreoVars.Poses.HELPL1);
    public static final Supplier<Pose2d> HELPL4 = flipped(ChoreoVars.Poses.HELPL4);
        public static final Supplier<Pose2d> HELPL5 = flipped(ChoreoVars.Poses.HELPL5);


    public static final Supplier<Pose2d> HELPR1 = flipped(ChoreoVars.Poses.HELPR1);
    public static final Supplier<Pose2d> HELPR4 = flipped(ChoreoVars.Poses.HELPR4);
    public static final Supplier<Pose2d> HELPR5 = flipped(ChoreoVars.Poses.HELPR5);


    // Climb poses
    public static final Supplier<Pose2d> CL1 = flipped(ChoreoVars.Poses.CL1);
    public static final Supplier<Pose2d> CL2 = flipped(ChoreoVars.Poses.CL2);

    // Depot poses
    public static final Supplier<Pose2d> DEPOT_START = flipped(ChoreoVars.Poses.DEPOTSTART);
    public static final Supplier<Pose2d> DEPOT_END = flipped(ChoreoVars.Poses.DEPOTEND);
    public static final Supplier<Pose2d> DEPOT_HELP = flipped(ChoreoVars.Poses.HELPD1);

    // Other poses
    public static final Supplier<Pose2d> STA1 = flipped(ChoreoVars.Poses.STA1);
    public static final Supplier<Pose2d> HELPS = flipped(ChoreoVars.Poses.HELPS);

    public static final Supplier<Pose2d> HUB1 = flipped(new Pose2d(4.625, 4.05, new Rotation2d()));

    public static final Supplier<Pose2d> BUMPHELP1 = flipped(ChoreoVars.Poses.BUMPHELP1);
    public static final Supplier<Pose2d> BUMPHELP2 = flipped(ChoreoVars.Poses.BUMPHELP2);

    public static final Supplier<Pose2d> CIRCLE_STOPL0 = flipped(ChoreoVars.Poses.CIRCLE_STOPL0);
    public static final Supplier<Pose2d> CIRCLE_STOPR0 = flipped(ChoreoVars.Poses.CIRCLESTOPR0);

    public static final Supplier<Pose2d> R_SecondSwipeStart = flipped(ChoreoVars.Poses.R_SecondSwipeStart);
    public static final Supplier<Pose2d> R_SecondSwipeStop = flipped(ChoreoVars.Poses.R_SecondSwipeStop);


    public static final Supplier<Pose2d> CIRCLE_STOPL101 = flipped(ChoreoVars.Poses.CIRCLESTOPL101);
    public static final Supplier<Pose2d> CIRCLE_STOPR101 = flipped(ChoreoVars.Poses.CIRCLESTOPR101);

    public static final Supplier<Pose2d> R_ScoreStop = flipped(ChoreoVars.Poses.R_ScoreStop);

    public static final Supplier<Pose2d> R_PassPathStop = flipped(ChoreoVars.Poses.R_PassPathStop);
    public static final Supplier<Pose2d> L_PassPathStop = flipped(ChoreoVars.Poses.L_PassPathStop);

    public static final Supplier<Pose2d> R_PASSHELP = flipped(ChoreoVars.Poses.R_PASSHELP);
    public static final Supplier<Pose2d> L_PASSHELP = flipped(ChoreoVars.Poses.L_PASSHELP);

    public static final Supplier<Pose2d> R_PASS_AUTO_STOP = flipped(ChoreoVars.Poses.R_PASS_AUTO_STOP);
    public static final Supplier<Pose2d> L_PASS_AUTO_STOP = flipped(ChoreoVars.Poses.L_PASS_AUTO_STOP);

    // L_SWEEP path poses

    public static final Supplier<Pose2d> L_SWEEP0 = flipped(ChoreoVars.Poses.L_SWEEP0);

    public static final Supplier<Pose2d> L_SWEEP6 = flipped(ChoreoVars.Poses.L_SWEEP6);
    public static final Supplier<Pose2d> L_SWEEP100 = flipped(ChoreoVars.Poses.L_SWEEP100);
    public static final Supplier<Pose2d> L_SWEEP5 = flipped(ChoreoVars.Poses.L_SWEEP5);
    public static final Supplier<Pose2d> L_SWEEP4 = flipped(ChoreoVars.Poses.L_SWEEP4);
    public static final Supplier<Pose2d> L_SWEEP3 = flipped(ChoreoVars.Poses.L_SWEEP3);
    public static final Supplier<Pose2d> L_SWEEP2 = flipped(ChoreoVars.Poses.L_SWEEP2);

    public static final Supplier<Pose2d> R_SWEEP6 = flipped(ChoreoVars.Poses.R_SWEEP6);
    public static final Supplier<Pose2d> R_SWEEP1 = flipped(ChoreoVars.Poses.R_SWEEP1);

    // ============= ROTATIONS =============

    // Center line intake pose rotations
    public static final Supplier<Rotation2d> BALLL2Entry = flipped(new Rotation2d(Degrees.of(-25)));
    public static final Supplier<Rotation2d> BALLL2CloseEntry = flipped(new Rotation2d(Degrees.of(0)));
    public static final Supplier<Rotation2d> BALLL4Entry = flipped(new Rotation2d(Degrees.of(-70)));

    public static final Supplier<Rotation2d> BALLR2Entry = flipped(new Rotation2d(Degrees.of(35)));
    public static final Supplier<Rotation2d> BALLR2CloseEntry = flipped(new Rotation2d(Degrees.of(180)));
    public static final Supplier<Rotation2d> BALLR4Entry = flipped(new Rotation2d(Degrees.of(90)));

    // Auto start pose rotations
    public static final Supplier<Rotation2d> TRL1Entry = flipped(new Rotation2d(Degrees.of(185)));

    public static final Supplier<Rotation2d> TRR1Entry = flipped(new Rotation2d(Degrees.of(175)));
        public static final Supplier<Rotation2d> TRR1CloseEntry = flipped(new Rotation2d(Degrees.of(180)));


    // Autoalign help pose rotations
    public static final Supplier<Rotation2d> HELPL1Entry = flipped(new Rotation2d(Degrees.of(0)));
    public static final Supplier<Rotation2d> HELPR2PassEntry = flipped(new Rotation2d(Degrees.of(250)));
    public static final Supplier<Rotation2d> HELPL2PassEntry = flipped(new Rotation2d(Degrees.of(-250)));

    public static final Supplier<Rotation2d> HELPL2Entry = flipped(new Rotation2d(Degrees.of(130)));
    public static final Supplier<Rotation2d> HELPL2CloseEntry = flipped(new Rotation2d(Degrees.of(140)));
    public static final Supplier<Rotation2d> HELPD1ntry = flipped(new Rotation2d(Degrees.of(160)));

    public static final Supplier<Rotation2d> HELPR1Entry = flipped(new Rotation2d(Degrees.of(-30)));
    public static final Supplier<Rotation2d> HELPR2Entry = flipped(new Rotation2d(Degrees.of(-130)));
    public static final Supplier<Rotation2d> HELPR2CloseEntry = flipped(new Rotation2d(Degrees.of(-140)));

    // Climb pose rotations
    // Other rotations
    public static final Supplier<Rotation2d> testEntry = flipped(new Rotation2d(Degrees.of(-90)));
    public static final Supplier<Rotation2d> depotStartEntry = flipped(new Rotation2d(Degrees.of(200)));

    public static final Supplier<Rotation2d> bumpToTrenchEntry = flipped(new Rotation2d(Degrees.of(0)));

    // Passing zones
    private static final Translation2d topZoneCorner1 = new Translation2d(FieldSize.FIELD_LENGTH, Meters.of(7.6));
    private static final Translation2d topZoneCorner2 = new Translation2d(Meters.of(4.7), Meters.of(4.7));
    private static final Translation2d centerZoneCorner1 = topZoneCorner2;
    private static final Translation2d centerZoneCorner2 = new Translation2d(FieldSize.FIELD_LENGTH, Meters.of(3.3));
    private static final Translation2d bottomZoneCorner1 = new Translation2d(Meters.of(4.7), Meters.of(0.5));
    private static final Translation2d bottomZoneCorner2 = centerZoneCorner2;

    private static final Translation2d allianceZoneCorner1 = new Translation2d(Meters.of(0), Meters.of(0));
    private static final Translation2d allianceZoneCorner2 = new Translation2d(Meters.of(4), Meters.of(8));
    private static final Translation2d topAllianceZoneCorner1 = new Translation2d(Meters.of(2.3), Meters.of(6));
    private static final Translation2d topAllianceZoneCorner2 = new Translation2d(Meters.of(2.3), Meters.of(6));
    private static final Translation2d bottomAllianceZoneCorner1 = new Translation2d(Meters.of(2.3), Meters.of(2));
    private static final Translation2d bottomAllianceZoneCorner2 = new Translation2d(Meters.of(2.3), Meters.of(2));

    public static final Supplier<Rectangle2d> allianceZone = flippedRectangle(allianceZoneCorner1, allianceZoneCorner2);
    public static final Supplier<Rectangle2d> topAllianceZone = flippedRectangle(topAllianceZoneCorner1,
            topAllianceZoneCorner2);
    public static final Supplier<Rectangle2d> bottomAllianceZone = flippedRectangle(bottomAllianceZoneCorner1,
            bottomAllianceZoneCorner2);

    public static final Supplier<Rectangle2d> topZone = flippedRectangle(topZoneCorner1, topZoneCorner2);
    public static final Supplier<Rectangle2d> centerZone = flippedRectangle(centerZoneCorner1, centerZoneCorner2);
    public static final Supplier<Rectangle2d> bottomZone = flippedRectangle(bottomZoneCorner1, bottomZoneCorner2);

    // ============= TRANSLATIONS =============
    public static final Supplier<Pose2d> bottomPassingPoint = flipped(
            new Pose2d(Meters.of(2.9), Meters.of(1.68), new Rotation2d()));
    public static final Supplier<Pose2d> topPassingPoint = flipped(
            new Pose2d(Meters.of(2.9), Meters.of(6.32), new Rotation2d()));

    // ============= DISTANCES =============
    public static final Distance kOriginToTrenchBlue = Meters.of(4.6);
    public static final Distance kOriginToTrenchRed = Meters.of(11.9);

    private static Supplier<Rectangle2d> flippedRectangle(Translation2d corner1, Translation2d corner2) {
        return AllianceFlipUtil.constant(new Rectangle2d(corner1, corner2),
                new Rectangle2d(ChoreoAllianceFlipUtil.flip(corner1), ChoreoAllianceFlipUtil.flip(corner2)));
    }

    // ============= PRIVATE RECTANGLES =============
    /**
     * Number of meters over the center/side lines acceptable for the auto game
     * piece pickup.
     */
    private static final double kLineToleranceMeters = 0.5;
    public static final Rectangle2d kLeftBlueBounds = new Rectangle2d(
            new Translation2d(
                    AllianceFlipUtil.FIELD_LENGTH / 2.0 + kLineToleranceMeters,
                    AllianceFlipUtil.FIELD_WIDTH / 2.0 - kLineToleranceMeters),
            new Translation2d(
                    6, AllianceFlipUtil.FIELD_WIDTH - kLineToleranceMeters));
    private static final Rectangle2d kRightBlueBounds = new Rectangle2d(
            new Translation2d(
                    6, kLineToleranceMeters),
            new Translation2d(
                    AllianceFlipUtil.FIELD_LENGTH / 2.0 + kLineToleranceMeters,
                    AllianceFlipUtil.FIELD_WIDTH / 2.0 + kLineToleranceMeters));
    public static final Rectangle2d kLeftRedBounds = new Rectangle2d(
            new Translation2d(
                    AllianceFlipUtil.FIELD_LENGTH, 0),
            new Translation2d(
                    AllianceFlipUtil.FIELD_LENGTH / 2.0 - kLineToleranceMeters,
                    AllianceFlipUtil.FIELD_WIDTH / 2.0 + kLineToleranceMeters));
    private static final Rectangle2d kRightRedBounds = new Rectangle2d(
            new Translation2d(
                    AllianceFlipUtil.FIELD_LENGTH, AllianceFlipUtil.FIELD_WIDTH - kLineToleranceMeters),
            new Translation2d(
                    AllianceFlipUtil.FIELD_LENGTH / 2.0 - kLineToleranceMeters,
                    AllianceFlipUtil.FIELD_WIDTH / 2.0 - kLineToleranceMeters));

    // ============= RECTANGLES =============
    public static final Supplier<Rectangle2d> kLeftAutoBounds = AllianceFlipUtil.constant(kLeftBlueBounds,
            kLeftRedBounds);
    public static final Supplier<Rectangle2d> kRightAutoBounds = AllianceFlipUtil.constant(kRightBlueBounds,
            kRightRedBounds);
    public static final Rectangle2d kFieldBounds = new Rectangle2d(new Translation2d(0, 0),
            new Translation2d(AllianceFlipUtil.FIELD_LENGTH, AllianceFlipUtil.FIELD_WIDTH));

    public static final Supplier<Rectangle2d> towerZone = flippedRectangle(
            new Translation2d(Meters.zero(), Meters.of(4.5)),
            new Translation2d(Meters.of(1.07), Meters.of(2.84)));
}