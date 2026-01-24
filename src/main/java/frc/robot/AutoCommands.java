package frc.robot;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.wpilibj2.command.Commands.*; // Static import for WPILib Commands

public class AutoCommands {
    // You need these dependencies passed in
    private final CommandSwerveDrivetrain m_drivebase;
    private final Autos autos; // Reference to your Autos class

    public AutoCommands(CommandSwerveDrivetrain drivebase, Autos autos) {
        this.m_drivebase = drivebase;
        this.autos = autos;
    }

    //CHOREO HAS NOT BEEN TESTED!
    /**
     * Runs a choreo path until a condition is met, then interrupts it
     */
    public Command runChoreoUntil(AutoTrajectory choreoPath, BooleanSupplier interruptCondition) {
        return race(
                // The choreo path to run
                choreoPath.cmd(),

                // The interruption logic
                sequence(
                        // Wait for the interrupt condition
                        waitUntil(interruptCondition),
                        // When condition met, print and cancel the choreo
                        runOnce(() -> {
                            choreoPath.cmd().cancel(); // Explicit cancel
                        })));
    }

    /**
     * Runs choreo until near a target pose
     */
    public Command runChoreoUntilNear(AutoTrajectory choreoPath, Pose2d targetPose,
            double toleranceMeters) {
        return runChoreoUntil(choreoPath, () -> {
            Pose2d currentPose = m_drivebase.getState().Pose;
            double distance = currentPose.getTranslation()
                    .getDistance(targetPose.getTranslation());
            return distance <= toleranceMeters;
        });
    }

    /**
     * Runs an AP command until a condition is met, then interrupts it
     */
    public Command runAPUntil(Command apCommand, BooleanSupplier interruptCondition) {
        return race(
                // The AP command to run
                apCommand,

                // The interruption logic
                sequence(
                        // Wait for the interrupt condition
                        waitUntil(interruptCondition),
                        // When condition met, print and cancel the AP
                        runOnce(() -> {
                            apCommand.cancel(); // Explicit cancel
                        })));
    }

    /**
     * AP command with timeout
     */
    public Command runAPWithTimeout(Command apCommand, double timeoutSeconds) {
        return deadline(
                waitSeconds(timeoutSeconds)

                ,
                apCommand);
    }

    /**
     * Runs AP to a pose until near the target (early termination)
     */
    public Command runDefaultAPUntilNear(Pose2d targetPose, double toleranceMeters) {
        // Use the autos instance to get the defaultAlignRequest
        Command apCommand = autos.defaultAlignRequest(targetPose);

        return race(
                apCommand,
                sequence(
                        waitUntil(() -> {
                            Pose2d currentPose = m_drivebase.getState().Pose;
                            double distance = currentPose.getTranslation()
                                    .getDistance(targetPose.getTranslation());
                            return distance <= toleranceMeters;
                        }),
                        runOnce(() -> {
                            apCommand.cancel();
                        })));
    }

}