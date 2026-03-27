package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.FlyWheelS;
import frc.robot.subsystems.hood.HoodS;
import frc.robot.subsystems.indexer.IndexerS;
import frc.robot.subsystems.intakepivot.IntakePivotS;
import frc.robot.subsystems.intakeroller.IntakeRollerS;
import frc.robot.subsystems.spindexer.SpindexerS;
import frc.robot.subsystems.turret.TurretS;
import frc.robot.subsystems.vision.detection.ObjectVision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoAlign;
import frc.robot.util.POI;
import frc.robot.util.TriggerUtil;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Autos {

    public class AutoConstants {
        private static Time kDefaultautoScoreTime = Seconds.of(2.0);
        private static Time kGPDTimeout = Seconds.of(6.0);
        private static Distance kGPDStartRadius = Meters.of(2.0);
        private static Distance kDefaultBackToStartRadius = Meters.of(12.8);
        private static Distance kCloseBackToStartRadius = Meters.of(2.0);
        private static Distance kDefaultStartRadius = Meters.of(3.5);
        private static Distance kDefaultBeginIntakingRadius = Meters.of(1.0);

    }

    private final AutoCommands autoCommands;
    private final AutoFactory factory;
    private final CommandSwerveDrivetrain m_drivebase;
    private final Map<String, Supplier<Command>> autos = new LinkedHashMap<>();
    private final HoodS m_hood;
    private final IntakePivotS m_intakePivot;
    private final IntakeRollerS m_intakeRoller;
    private final TurretS m_turret;
    private final IndexerS m_indexer;
    private final SpindexerS m_spindexer;
    private final FlyWheelS m_FlyWheel;
    private final ObjectVision m_objectVision;

    public Autos(AutoCommands autoCommands, CommandSwerveDrivetrain drive, AutoFactory factory,
            RobotContainer container, HoodS hood,
            IntakePivotS intakePivot, IntakeRollerS intakeRoller, TurretS turret, IndexerS indexer,
            SpindexerS spindexer, FlyWheelS flyWheel, ObjectVision objectVision) {
        this.factory = factory;
        this.autoCommands = autoCommands;
        this.m_hood = hood;
        this.m_intakePivot = intakePivot;
        this.m_intakeRoller = intakeRoller;
        this.m_turret = turret;
        this.m_drivebase = drive;
        this.m_indexer = indexer;
        this.m_spindexer = spindexer;
        this.m_FlyWheel = flyWheel;
        this.m_objectVision = objectVision;
        // ============= CHOREO PATHS =============
        Command run = factory.trajectoryCmd("Poses");
        Command L_Sweep = factory.trajectoryCmd("L_Sweep");
        Command R_Sweep = factory.trajectoryCmd("R_Sweep");

        // ============= PREDEFINED HELPERS =============

        // (L/R) Center Line Preplanned
        Supplier<Command> leftToCenterLineMiddleHardCoded = () -> autoCommands.APToIntake(POI.HELPL1.get(),
                AutoConstants.kDefaultStartRadius,
                POI.BALLL2.get(), POI.BALLL2Entry.get(), AutoConstants.kDefaultBeginIntakingRadius,
                POI.STOPL1.get());

        Supplier<Command> leftToCenterLineCloseHardCoded = () -> autoCommands.APToIntake(POI.HELPL1.get(),
                AutoConstants.kDefaultStartRadius,
                POI.BALLL3.get(), POI.BALLL4Entry.get(), AutoConstants.kDefaultBeginIntakingRadius,
                POI.STOPL2.get());

        Supplier<Command> rightToCenterLineMiddleHardCoded = () -> autoCommands.APToIntake(POI.HELPR1.get(),
                AutoConstants.kDefaultStartRadius,
                POI.BALLR2.get(), POI.BALLR2Entry.get(), AutoConstants.kDefaultBeginIntakingRadius,
                POI.STOPR1.get());

        Supplier<Command> rightToCenterLineCloseHardCoded = () -> autoCommands.APToIntake(POI.HELPR1.get(),
                AutoConstants.kDefaultStartRadius,
                POI.BALLR3.get(), POI.BALLR4Entry.get(), AutoConstants.kDefaultBeginIntakingRadius,
                POI.STOPR2.get());

        // (L/R) Center Line Middle GPD
        Supplier<Command> leftToCenterLineGPD = () -> leftToCenterLineMiddleHardCoded.get()
                .until(() -> m_objectVision.getBestCluster().isPresent()
                        && TriggerUtil.isWithinRadius(() -> POI.BALLL3.get().getTranslation(),
                                () -> m_drivebase.state.Pose,
                                () -> AutoConstants.kGPDStartRadius).getAsBoolean())

                .andThen(autoCommands.APToClusterChain(80000, true))
                .withTimeout(AutoConstants.kGPDTimeout);

        Supplier<Command> rightToCenterLineGPD = () -> rightToCenterLineMiddleHardCoded.get()
                .until(() -> m_objectVision.getBestCluster().isPresent()
                        && TriggerUtil.isWithinRadius(() -> POI.BALLL3.get().getTranslation(),
                                () -> m_drivebase.state.Pose,
                                () -> AutoConstants.kGPDStartRadius).getAsBoolean())
                .andThen(autoCommands.APToClusterChain(90000, false))
                .withTimeout(AutoConstants.kGPDTimeout);

        // (L/R) Back From Center Line Default
        Supplier<Command> leftBackToStartDefault = () -> autoCommands.APBackFromIntake(POI.HELPL4.get(),
                POI.HELPL2Entry.get(), Meters.of(3),
                POI.TRL1.get(), POI.TRL1Entry.get());

        Supplier<Command> leftBackToStartClose = () -> autoCommands.APBackFromIntake(POI.HELPL4.get(),
                POI.HELPL2Entry.get(), AutoConstants.kCloseBackToStartRadius,
                POI.TRL1.get(), POI.TRL1Entry.get());

        Supplier<Command> rightBackToStartDefault = () -> autoCommands.APBackFromIntake(POI.HELPR2.get(),
                POI.HELPR2Entry.get(), AutoConstants.kDefaultBackToStartRadius,
                POI.TRR1.get(), POI.TRR1Entry.get());

        Supplier<Command> rightBackToStartClose = () -> autoCommands.APBackFromIntake(POI.HELPR4.get(),
                POI.HELPL2Entry.get(), AutoConstants.kCloseBackToStartRadius,
                POI.TRR1.get(), POI.TRL1Entry.get());

        // (L/R) Sweep Default
        Supplier<Command> leftStartSweepDefault = () -> autoCommands.APToIntake(POI.HELPL1.get(),
                AutoConstants.kDefaultStartRadius,
                POI.BALLL2.get(), POI.BALLL2Entry.get(), AutoConstants.kDefaultStartRadius,
                POI.STOPL3.get())
                .andThen(new AutoAlign(POI.TRR2.get(), POI.TRR1Entry.get(), m_drivebase,
                        AutoAlign.kDefaultVelocityLimitedProfile));

        Supplier<Command> rightStartSweepDefault = () -> autoCommands.APToIntake(POI.HELPR1.get(),
                AutoConstants.kDefaultStartRadius,
                POI.BALLR2.get(), POI.BALLR2Entry.get(), AutoConstants.kDefaultStartRadius,
                POI.STOPR3.get())
                .andThen(new AutoAlign(POI.TRL2.get(), POI.TRL1Entry.get(), m_drivebase,
                        AutoAlign.kDefaultVelocityLimitedProfile));

        // (L) Back From Center Line To Depot
        Supplier<Command> leftBackToDepot = () -> autoCommands.APBackFromIntake(POI.HELPL3.get(),
                POI.HELPL2Entry.get(), Meters.of(12.2),
                POI.DEPOT_HELP.get(), POI.TRL1Entry.get())
                .until(
                        TriggerUtil.isWithinRadius(
                                () -> POI.DEPOT_HELP.get()
                                        .getTranslation(),
                                () -> m_drivebase.state.Pose,
                                () -> Meters.of(0.1)))
                .andThen(

                        autoCommands.autoScore()
                                .withTimeout(AutoConstants.kDefaultautoScoreTime)
                                .alongWith(autoCommands.APtoDepot()));

        Supplier<Command> rightBackToHP = () -> autoCommands.APBackFromIntake(POI.HELPR3.get(),
                POI.HELPR2Entry.get(), Meters.of(12.5),
                POI.HELPS.get(), POI.TRL1Entry.get())
                .until(
                        TriggerUtil.isWithinRadius(
                                () -> POI.HELPS.get()
                                        .getTranslation(),
                                () -> m_drivebase.state.Pose,
                                () -> Meters.of(0.3)))
                .andThen(

                        autoCommands.autoScore()
                                .withTimeout(AutoConstants.kDefaultautoScoreTime)
                                .alongWith(new AutoAlign(POI.STA1.get(), m_drivebase,
                                        AutoAlign.kSlowDriveProfile)));
        // (L/R) Back From Center Line To Climb
        Supplier<Command> leftChoreoSweepBack = () -> L_Sweep.until(TriggerUtil.isWithinRadius(
                () -> POI.L_SWEEP6.get()
                        .getTranslation(),
                () -> m_drivebase.state.Pose,
                () -> Meters.of(0.2)))
                .andThen(new AutoAlign(POI.TRL2.get(), POI.TRL1Entry.get(), m_drivebase,
                        AutoAlign.kDefaultVelocityLimitedProfile));

        Supplier<Command> rightChoreoSweepBack = () -> R_Sweep.until(TriggerUtil.isWithinRadius(
                () -> POI.R_SWEEP6.get()
                        .getTranslation(),
                () -> m_drivebase.state.Pose,
                () -> Meters.of(0.2)))
                .andThen(new AutoAlign(POI.TRR2.get(), POI.TRL1Entry.get(), m_drivebase,
                        AutoAlign.kDefaultVelocityLimitedProfile));

        Supplier<Command> leftCircleStart = () -> autoCommands.APToIntake(POI.HELPL1.get(),
                AutoConstants.kDefaultStartRadius,
                POI.BALLL2.get(), POI.BALLL2Entry.get(), AutoConstants.kDefaultBeginIntakingRadius,
                POI.CIRCLE_STOPL0.get());

        Supplier<Command> rightCircleStart = () -> autoCommands.APToIntake(POI.HELPR1.get(),
                AutoConstants.kDefaultStartRadius,
                POI.BALLR2.get(), POI.BALLR2Entry.get(), AutoConstants.kDefaultBeginIntakingRadius,
                POI.CIRCLE_STOPR0.get());

        Supplier<Command> leftCircleMid = () -> autoCommands.APToIntake(POI.HELPL1.get(),
                AutoConstants.kDefaultStartRadius,
                POI.BALLL3.get(), POI.BALLL4Entry.get(), AutoConstants.kDefaultBeginIntakingRadius,
                POI.CIRCLE_STOPL101.get());

        Supplier<Command> rightCircleMid = () -> autoCommands.APToIntake(POI.HELPR1.get(),
                AutoConstants.kDefaultStartRadius,
                POI.BALLR3.get(), POI.BALLR4Entry.get(), AutoConstants.kDefaultBeginIntakingRadius,
                POI.CIRCLE_STOPR101.get());

        Supplier<Command> leftCircleOverBumpAndScore = () -> new AutoAlign(POI.BUMPHELP1.get(), m_drivebase,
                AutoAlign.kDefaultVelocityLimitedProfile)

                .until(TriggerUtil.isWithinRadius(
                        () -> POI.BUMPHELP1.get()
                                .getTranslation(),
                        () -> m_drivebase.state.Pose,
                        () -> Meters.of(0.5)))

                .andThen(autoCommands.driveOverBump(true))

                .andThen(Commands.race(new AutoAlign(POI.TRL1.get(), POI.bumpToTrenchEntry.get(), m_drivebase,
                        AutoAlign.kSlowCrawlProfile),
                        autoCommands.Score()));

        Supplier<Command> rightCircleOverBumpAndScore = () -> new AutoAlign(POI.BUMPHELP2.get(), m_drivebase,
                AutoAlign.kDefaultVelocityLimitedProfile)

                .until(TriggerUtil.isWithinRadius(
                        () -> POI.BUMPHELP2.get()
                                .getTranslation(),
                        () -> m_drivebase.state.Pose,
                        () -> Meters.of(0.5)))

                .andThen(autoCommands.driveOverBump(true))

                .andThen(Commands.race(new AutoAlign(POI.TRR1.get(), POI.bumpToTrenchEntry.get(), m_drivebase,
                        AutoAlign.kSlowCrawlProfile), autoCommands.Score()));

        // (L/R) Back From Center Line To Climb

        // ============= DEFINE AUTOS =============

        autos.put("L 2x trench",
                () -> auto(POI.TRL1.get(), c -> {
                    c.addCommands(leftToCenterLineMiddleHardCoded.get());

                    c.addCommands(leftBackToStartDefault.get());

                    c.addCommands(autoCommands.Score()
                            .withTimeout(AutoConstants.kDefaultautoScoreTime));

                    c.addCommands(autoCommands.autoScore()
                            .withTimeout(AutoConstants.kDefaultautoScoreTime));

                    c.addCommands(leftToCenterLineCloseHardCoded.get()
                            .until(TriggerUtil.isWithinRadius(
                                    () -> POI.L_SWEEP0.get()
                                            .getTranslation(),
                                    () -> m_drivebase.state.Pose,
                                    () -> Meters.of(0.4))));

                    c.addCommands(leftChoreoSweepBack.get());

                    c.addCommands(autoCommands.Score()
                            .withTimeout(AutoConstants.kDefaultautoScoreTime));

                    c.addCommands(autoCommands.autoScore());

                }));

        autos.put("R 2x trench",
                () -> auto(POI.TRR1.get(), c -> {
                    c.addCommands(rightToCenterLineMiddleHardCoded.get());

                    c.addCommands(rightBackToStartDefault.get());

                    c.addCommands(autoCommands.Score()
                            .withTimeout(AutoConstants.kDefaultautoScoreTime));

                    c.addCommands(autoCommands.autoScore()
                            .withTimeout(AutoConstants.kDefaultautoScoreTime));

                    c.addCommands(rightToCenterLineCloseHardCoded.get()
                            .until(TriggerUtil.isWithinRadius(
                                    () -> POI.R_SWEEP1.get()
                                            .getTranslation(),
                                    () -> m_drivebase.state.Pose,
                                    () -> Meters.of(0.4))));

                    c.addCommands(rightChoreoSweepBack.get());

                    c.addCommands(autoCommands.Score()
                            .withTimeout(AutoConstants.kDefaultautoScoreTime));

                    c.addCommands(autoCommands.autoScore());
                }));

        autos.put("L 2.5x bump", () -> auto(POI.TRL1.get(), c -> {
            c.addCommands(leftCircleStart.get());

            c.addCommands(leftCircleOverBumpAndScore.get());

            c.addCommands(leftCircleMid.get());

            c.addCommands(leftCircleOverBumpAndScore.get());

            c.addCommands(leftCircleMid.get());

            c.addCommands(leftCircleOverBumpAndScore.get());

        }));

        autos.put("R 2.5x bump", () -> auto(POI.TRR1.get(), c -> {
            c.addCommands(rightCircleStart.get());

            c.addCommands(rightCircleOverBumpAndScore.get());

            c.addCommands(rightCircleMid.get());

            c.addCommands(rightCircleOverBumpAndScore.get());

            c.addCommands(rightCircleMid.get());

            c.addCommands(rightCircleOverBumpAndScore.get());

        }));

        autos.put("Seeding-test", () -> auto(POI.CL1.get(), c -> {
            c.addCommands(Commands.none());
        }));
        // Auto-register
        autos.forEach((name, sup) -> container.m_chooser.addCmd(name, sup));

    }
    // ============= FLEXIBLE AUTO BUILDER =============

    /**
     * Build any auto with command flexibility
     * 
     * @param startPose Starting pose (auto-resets odometry)
     * @param builder   A consumer that builds the command sequence using
     *                  addCommands()
     */
    private Command auto(Pose2d startPose, Consumer<SequentialCommandGroup> builder) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        // Odometry reset first
        group.addCommands(factory.resetOdometry(Optional.of(startPose), false));

        // Let the builder add more commands
        builder.accept(group);
        return group;
    }

}