package frc.robot.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.util.AutoAlign;
import frc.robot.util.POI;
import frc.robot.util.TriggerUtil;
import frc.robot.RobotContainer;

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
            SpindexerS spindexer, FlyWheelS flyWheel) {
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

        // ============= PREDEFINED HELPERS =============
        Supplier<Command> defaultAPToCenterLineCommandLeft = () -> autoCommands.APToIntake(POI.HELPL1.get(),
                Meters.of(3.5),
                POI.BALLL2.get(), POI.BALLL2Entry.get(), Meters.of(0.12), POI.STOPL1.get());

        Supplier<Command> defaultAPBackToStartCommandLeft = () -> autoCommands.APBackFromIntake(POI.HELPL2.get(),
                POI.HELPL2Entry.get(), Meters.of(1.8),
                POI.TRL1.get(), POI.TRL1Entry.get());

        // ============= CHOREO PATHS =============
        Command run = factory.trajectoryCmd("Poses");

        // ============= DEFINE AUTOS =============
        autos.put("L center-line 1x tune",
                () -> auto(POI.TRL1.get(), c -> {
                    c.addCommands(defaultAPToCenterLineCommandLeft.get());

                    c.addCommands(defaultAPBackToStartCommandLeft.get());

                    c.addCommands(autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime));
                }));

        autos.put("2x test",
                () -> auto(POI.TRL1.get(), c -> {
                    c.addCommands(defaultAPToCenterLineCommandLeft.get());

                    c.addCommands(defaultAPBackToStartCommandLeft.get());

                    c.addCommands(autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime));

                    c.addCommands(autoCommands.APToIntake(POI.HELPL1.get(), Meters.of(3.5), POI.BALLL3.get(),
                            POI.BALLL2Entry.get(), Meters.of(0.1), POI.STOPL2.get()));

                    c.addCommands(
                            autoCommands.APBackFromIntake(POI.HELPL2.get(), POI.HELPL2CloseEntry.get(), Meters.of(2.0),
                                    POI.TRL1.get(), POI.TRL1Entry.get()));

                    c.addCommands((autoCommands.Score().withTimeout(AutoConstants.kDefaultAutoScoreTime)));
                }));

        autos.put("DEPOT L center-line 1x tune",
                () -> auto(POI.TRL1.get(), c -> {
                    c.addCommands(defaultAPToCenterLineCommandLeft.get());

                    c.addCommands(autoCommands
                            .APBackFromIntake(POI.HELPL2.get(), POI.HELPL2Entry.get(), Meters.of(1.2),
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