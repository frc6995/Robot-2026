package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
        private static Time kDefaultAutoScoreTime = Seconds.of(2.0);

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

    /*
     * . CHOREO AUTO EXAMPLE
     * 
     * String choreoAutoName = "Choreo Auto";
     * 
     * public AutoRoutine choreoAuto() {
     * final AutoRoutine routine = factory.newRoutine(choreoAutoName);
     * final AutoTrajectory traj = routine.trajectory("OP");
     * routine.active().onTrue(
     * traj.resetOdometry()
     * .andThen(traj.cmd()));
     * return routine;
     * }
     */
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

        // ============= PREDEFINED HELPERS =============

        // (L/R) Center Line Middle Preplanned
        Supplier<Command> leftToCenterLineMiddleHardCoded = () -> autoCommands.APToIntake(POI.HELPL1.get(),
                Meters.of(3.5),
                POI.BALLL2.get(), POI.BALLL2Entry.get(), Meters.of(0.12), POI.STOPL1.get());

        Supplier<Command> rightToCenterLineMiddleHardCoded = () -> autoCommands.APToIntake(POI.HELPR1.get(),
                Meters.of(3.5),
                POI.BALLR2.get(), POI.BALLR2Entry.get(), Meters.of(0.12), POI.STOPR1.get());

        // (L/R) Center Line Middle GPD
        Supplier<Command> leftToCenterLineGPD = () -> leftToCenterLineMiddleHardCoded.get()
                .until(() -> m_objectVision.getBestCluster().isPresent()
                        && TriggerUtil.isWithinTolerance(
                                () -> m_drivebase.state.Pose.getRotation().getDegrees(),
                                AllianceFlipUtil.constant(-90.0, 90.0),
                                () -> 15.0).getAsBoolean())
                .andThen(autoCommands.APToClusterChain(80000, true)).withTimeout(7.0);

        Supplier<Command> rightToCenterLineGPD = () -> rightToCenterLineMiddleHardCoded.get()
                .until(() -> m_objectVision.getBestCluster().isPresent()
                        && TriggerUtil.isWithinTolerance(
                                () -> m_drivebase.state.Pose.getRotation().getDegrees(),
                                AllianceFlipUtil.constant(90.0, -90.0),
                                () -> 15.0).getAsBoolean())
                .andThen(autoCommands.APToClusterChain(90000, false)).withTimeout(7.0);

        // (L/R) Back From Center Line Default
        Supplier<Command> leftBackToStartDefault = () -> autoCommands.APBackFromIntake(POI.HELPL2.get(),
                POI.HELPL2Entry.get(), Meters.of(3.0),
                POI.TRL1.get(), POI.TRL1Entry.get());

        Supplier<Command> rightBackToStartDefault = () -> autoCommands.APBackFromIntake(POI.HELPR2.get(),
                POI.HELPR2Entry.get(), Meters.of(3.0),
                POI.TRR1.get(), POI.TRR1Entry.get());

        // (L/R) Sweep Default
        Supplier<Command> leftStartSweepDefault = () -> autoCommands.APToIntake(POI.HELPL1.get(),
                Meters.of(3.5),
                POI.BALLL2.get(), POI.BALLL2Entry.get(), Meters.of(0.12), POI.STOPL3.get())
                .andThen(new AutoAlign(POI.TRR2.get(), POI.TRR1Entry.get(), m_drivebase,
                        AutoAlign.kDefaultVelocityLimitedProfile));

        Supplier<Command> rightStartSweepDefault = () -> autoCommands.APToIntake(POI.HELPR1.get(),
                Meters.of(3.5),
                POI.BALLR2.get(), POI.BALLR2Entry.get(), Meters.of(0.12), POI.STOPR3.get())
                .andThen(new AutoAlign(POI.TRL2.get(), POI.TRL1Entry.get(), m_drivebase,
                        AutoAlign.kDefaultVelocityLimitedProfile));

        // (L) Back From Center Line To Depot
        Supplier<Command> leftBackToStartDefaultPlusDepot = () ->autoCommands.APBackFromIntake(POI.HELPL3.get(),
                POI.HELPL2Entry.get(), Meters.of(2.8),
                POI.DEPOT_HELP.get(), POI.TRL1Entry.get())
                            .until(
                                    TriggerUtil.isWithinRadius(
                                            () -> POI.DEPOT_HELP.get()
                                                    .getTranslation(),
                                            () -> m_drivebase.state.Pose,
                                            () -> Meters.of(0.1))).andThen(

                    autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime)
                            .alongWith(autoCommands.APtoDepot()));
                

        // (L/R) Back From Center Line To Climb

        // ============= CHOREO PATHS =============
        Command run = factory.trajectoryCmd("Poses");

        // ============= DEFINE AUTOS =============

        autos.put("L 3x center-line",
                () -> auto(POI.TRL1.get(), c -> {
                    c.addCommands(leftToCenterLineMiddleHardCoded.get());

                    c.addCommands(leftBackToStartDefault.get());

                    c.addCommands(autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime));

                    c.addCommands(leftToCenterLineGPD.get());

                    c.addCommands(leftBackToStartDefault.get());

                    c.addCommands(autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime));

                    c.addCommands(leftToCenterLineGPD.get());

                    c.addCommands(leftBackToStartDefault.get());

                    c.addCommands(autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime));
                }));

        autos.put("R 3x center-line",
                () -> auto(POI.TRR1.get(), c -> {
                    c.addCommands(rightToCenterLineMiddleHardCoded.get());

                    c.addCommands(rightBackToStartDefault.get());

                    c.addCommands(autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime));

                    c.addCommands(rightToCenterLineGPD.get());

                    c.addCommands(rightBackToStartDefault.get());

                    c.addCommands(autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime));

                    c.addCommands(rightToCenterLineGPD.get());

                    c.addCommands(rightBackToStartDefault.get());

                    c.addCommands(autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime));
                    // ADD GPD ONES
                }));

        autos.put("L 2x center-line + Depot",
                () -> auto(POI.TRL1.get(), c -> {
                    c.addCommands(leftToCenterLineMiddleHardCoded.get());

                     c.addCommands(leftBackToStartDefault.get());
                     c.addCommands(leftToCenterLineGPD.get());

                    c.addCommands(leftBackToStartDefaultPlusDepot.get());

                }));

        autos.put("L center-line 1x tune",
                () -> auto(POI.TRL1.get(), c -> {
                    c.addCommands(leftToCenterLineMiddleHardCoded.get());

                    c.addCommands(leftBackToStartDefault.get());

                    c.addCommands(autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime));
                }));

        autos.put("L Sweep test",
                () -> auto(POI.TRL1.get(), c -> {
                    c.addCommands(leftStartSweepDefault.get());

                    c.addCommands(autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime));

                }));

        autos.put("DEPOT L center-line 1x tune",
                () -> auto(POI.TRL1.get(), c -> {
                    c.addCommands(leftToCenterLineMiddleHardCoded.get());

                    c.addCommands(autoCommands.APBackFromIntake(POI.HELPL3.get(),
                POI.HELPL2Entry.get(), Meters.of(2.8),
                POI.DEPOT_HELP.get(), POI.TRL1Entry.get())
                            .until(
                                    TriggerUtil.isWithinRadius(
                                            () -> POI.DEPOT_HELP.get()
                                                    .getTranslation(),
                                            () -> m_drivebase.state.Pose,
                                            () -> Meters.of(1.7))));

                    c.addCommands(autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime)
                            .alongWith(autoCommands.APtoDepot()));
                }));

        autos.put("Depot test", () -> auto(POI.TRL1.get(), c -> {
            c.addCommands(new AutoAlign(POI.DEPOT_HELP.get(), m_drivebase));
            c.addCommands(autoCommands.APtoDepot());

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