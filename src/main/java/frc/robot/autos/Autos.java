package frc.robot.autos;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlyWheelS;
import frc.robot.subsystems.HoodS;
import frc.robot.subsystems.IndexerS;
import frc.robot.subsystems.IntakePivotS;
import frc.robot.subsystems.IntakeRollerS;
import frc.robot.subsystems.SpindexerS;
import frc.robot.subsystems.TurretS;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoAlign;
import frc.robot.generated.ChoreoTraj;
import frc.robot.util.ChoreoVariables;
import frc.robot.util.POI;
import yams.mechanisms.velocity.FlyWheel;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Autos {

        public class AutoConstants {
                private static double DEFAULT_ACCELERATION = 15;
                private static double DEFAULT_JERK = 12;

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
    public Autos(CommandSwerveDrivetrain drive, AutoFactory factory, RobotContainer container, HoodS hood,
            IntakePivotS intakePivot, IntakeRollerS intakeRoller, TurretS turret, IndexerS indexer, SpindexerS spindexer, FlyWheelS flyWheel) {
        this.factory = factory;
        autoCommands = new AutoCommands(drive, this, hood, intakePivot, intakeRoller, turret, indexer, spindexer,flyWheel);
        this.m_hood = hood;
        this.m_intakePivot = intakePivot;
        this.m_intakeRoller = intakeRoller;
        this.m_turret = turret;
        this.m_drivebase = drive;
        this.m_indexer = indexer;
        this.m_spindexer = spindexer;
        this.m_FlyWheel = flyWheel;
        // ============= DEFINE AUTOS =============
        Command run = factory.trajectoryCmd("Poses");

        // ============= DEFINE AUTOS =============

        autos.put("L center-line 2x", () -> auto(POI.TRL1.get(),
                autoCommands.APToIntake(POI.HELPL1.get(),
                                POI.HELPL1Entry.get(),
                                Meters.of(2.0),
                                POI.BALLL2.get(),
                                POI.BALLL2Entry.get(),
                                Meters.of(0.15),
                                Seconds.of(5)

                )

                                .andThen(autoCommands.APBackFromIntake(POI.HELPL2.get(),
                                                POI.HELPL2Entry.get(),
                                                Meters.of(1.4),
                                                POI.TRL1.get(),
                                                POI.TRL1Entry.get()

                                ))
                                .andThen(autoCommands.Score().withTimeout(Seconds.of(2)))
                                .andThen(autoCommands.APToIntake(POI.HELPL1.get(),
                                                POI.HELPL1Entry.get(),
                                                Meters.of(2.0),
                                                POI.BALLL3.get(),
                                                POI.BALLL2Entry.get(),
                                                Meters.of(0.15),
                                                Seconds.of(0.5)))
                                .andThen(autoCommands.APBackFromIntake(POI.HELPL2.get(),
                                                POI.HELPL2Entry.get(),
                                                Meters.of(2.1),
                                                POI.TRL1.get(),
                                                POI.TRL1Entry.get()

                                ))
                                .andThen(autoCommands.Score().withTimeout(Seconds.of(2)))));

        autos.put("Choreo-test", () -> auto(POI.TRL1.get(),
                        autoCommands.choreoToIntake(run, POI.HELPL1.get(),
                                        Meters.of(2.0),
                                        POI.BALLL2.get(),
                                        POI.BALLL2Entry.get(),
                                        Meters.of(0.15),
                                        Seconds.of(0.5))));
        // Auto-register
        autos.forEach((name, sup) -> container.m_chooser.addCmd(name, sup));

        }

        // ============= FLEXIBLE AUTO BUILDER =============

        /**
         * Build any auto with command flexibility
         * 
         * @param startPose Starting pose (auto-resets odometry)
         * @param command   Any sequence of commands (AP, choreo, actions, etc.)
         */
        private Command auto(Pose2d startPose, Command command) {
                return factory.resetOdometry((Optional.of(startPose)), false).andThen(new ScheduleCommand(command));
        }

}