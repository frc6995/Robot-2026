package frc.robot;

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
import frc.robot.util.ChoreoVariables;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Autos {

    public class AutoConstants {
        public static double DEFAULT_ACCELERATION = 15;
        public static double DEFAULT_JERK = 12;
    }

    private final AutoFactory factory;
    private final CommandSwerveDrivetrain m_drivebase;
    private final Map<String, Supplier<AutoRoutine>> autos = new LinkedHashMap<>();

    public Autos(CommandSwerveDrivetrain drive, AutoFactory factory, RobotContainer container) {
        this.factory = factory;
        this.m_drivebase = drive;

        // Define all autos - now with full flexibility
        autos.put("AP_Simple", () -> auto("AP_Simple", POI.testStart,
                defaultAlignRequest(POI.testEnd)));

        autos.put("AP_Multi", () -> auto("AP_Multi", POI.testStart,
                defaultAlignRequest(POI.testStart),
                defaultAlignRequest(POI.testStart)));
        autos.put("Hybrid_Flex", () -> auto("Hybrid_Flex", POI.testStart,
                defaultAlignRequest(POI.testStart),
                interruptibleChoreo("PrecisePath"),
                defaultAlignRequest(POI.testEnd),
                defaultAlignRequest(POI.testStart)));

        // Auto-register
        autos.forEach((name, sup) -> container.m_chooser.addRoutine(name, sup));
    }

    // ============= FLEXIBLE AUTO BUILDER =============

    /**
     * Build any auto with full command sequence flexibility
     * 
     * @param name      Auto name
     * @param startPose Starting pose (auto-resets odometry)
     * @param commands  Any sequence of commands (AP, choreo, actions, etc.)
     */
    private AutoRoutine auto(String name, Pose2d startPose, Command... commands) {
        AutoRoutine r = factory.newRoutine(name);

        // Start with odometry reset
        Command sequence = factory.resetOdometry(() -> Optional.of(startPose));

        // Add all provided commands
        for (Command cmd : commands) {
            sequence = sequence.andThen(cmd);
        }

        r.active().onTrue(sequence);
        return r;
    }

    // ============= REUSABLE COMMAND BUILDERS =============

    /**
     * Interruptible choreo path (can be cancelled by safety/operator)
     */
    private Command interruptibleChoreo(String choreoName) {
        // Create a temporary routine to get the trajectory
        AutoTrajectory choreo = factory.newRoutine("temp").trajectory(choreoName);

        return Commands.race(
                choreo.cmd(),
                Commands.waitUntil(this::shouldInterrupt)
                        .andThen(Commands.print("Interrupted choreo: " + choreoName)));
    }

    /**
     * Interruptible choreo with custom interruption condition
     */
    private Command interruptibleChoreo(String choreoName, Supplier<Boolean> interruptCondition) {
        AutoTrajectory choreo = factory.newRoutine("temp").trajectory(choreoName);

        return Commands.race(
                choreo.cmd(),
                Commands.waitUntil((BooleanSupplier) interruptCondition)
                        .andThen(Commands.print("Custom interrupt: " + choreoName)));
    }

    // ============= UTILITY COMMANDS =============

    private boolean shouldInterrupt() {
        // Add your interruption logic. Within tolerance?
        return false;
    }

    // ============= CONVENIENCE WRAPPERS =============

    /**
     * Quick AP-only auto builder for simple point-to-point
     */
    private AutoRoutine apOnlyAuto(String name, Pose2d start, Pose2d... targets) {
        Command[] commands = new Command[targets.length];
        for (int i = 0; i < targets.length; i++) {
            commands[i] = defaultAlignRequest(targets[i]);
        }
        return auto(name, start, commands);
    }

    /**
     * Hybrid auto builder for common pattern
     */
    private AutoRoutine hybridPatternAuto(String name, Pose2d start,
            String choreoName, Pose2d afterChoreo) {
        return auto(name, start,
                defaultAlignRequest(ChoreoVars.Poses.testStart),
                interruptibleChoreo(choreoName),
                defaultAlignRequest(afterChoreo));
    }

    /**
     * Creates a new Command using the Autopilot AutoAlign to navigate to the
     * targetPose.
     * 
     * @param targetPose The desired ending Pose2d
     * @return The Command to navigate to the given Pose2d.
     */
    public Command defaultAlignRequest(Pose2d targetPose) {
        return new AutoAlign(new APTarget(AllianceFlipUtil.flipPose(targetPose)), m_drivebase,
                new APConstraints(AutoConstants.DEFAULT_ACCELERATION, AutoConstants.DEFAULT_JERK));
    }

    /**
     * Creates a new Command using the Autopilot AutoAlign to navigate to the
     * targetPose. Takes a desired entry angle
     * when approaching the targetPose.
     * 
     * @param targetPose The desired ending Pose2d
     * @param entryAngle The desired angle to approach the targetPose with.
     * @return The Command to navigate to the given Pose2d.
     */
    public Command defaultAlignRequest(Pose2d targetPose, Rotation2d entryAngle) {
        return new AutoAlign(new APTarget(AllianceFlipUtil.flipPose(targetPose))
                .withEntryAngle(AllianceFlipUtil.flipRotation(entryAngle)), m_drivebase,
                new APConstraints(AutoConstants.DEFAULT_ACCELERATION, AutoConstants.DEFAULT_JERK));
    }


}