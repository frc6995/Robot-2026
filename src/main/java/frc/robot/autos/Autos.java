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
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoAlign;
import frc.robot.generated.ChoreoTraj;
import frc.robot.util.ChoreoVariables;
import frc.robot.util.POI;
import frc.robot.RobotContainer;

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
    public Autos(CommandSwerveDrivetrain drive, AutoFactory factory, RobotContainer container) {
        this.factory = factory;
        autoCommands = new AutoCommands(drive, this);
        this.m_drivebase = drive;

        // ============= DEFINE AUTOS =============
        Command run = factory.trajectoryCmd("Poses");

        autos.put("EntryAngle", () -> auto("EntryAngle", POI.CL1.get(),
                new AutoAlign(POI.HELPL1.get(), m_drivebase),
                run

        ));

        autos.put("Choreo test", () -> auto("Choreo test", POI.TRR1.get(),
                run

        ));

        // Auto-register
        autos.forEach((name, sup) -> container.m_chooser.addCmd(name, sup));
        // Choreo Auto
        // container.m_chooser.addRoutine(choreoAutoName, this::choreoAuto);

    }

    // ============= FLEXIBLE AUTO BUILDER =============

    /**
     * Build any auto with command sequence flexibility
     * 
     * @param name      Auto name
     * @param startPose Starting pose (auto-resets odometry)
     * @param commands  Any sequence of commands (AP, choreo, actions, etc.)
     */
    private Command auto(String name, Pose2d startPose, Command... commands) {

        // Start with odometry reset
        Command sequence = factory.resetOdometry((Optional.of(startPose)), false);

        // Add all provided commands
        for (Command cmd : commands) {
            sequence = sequence.andThen(cmd);
        }

        return sequence;
    }

}