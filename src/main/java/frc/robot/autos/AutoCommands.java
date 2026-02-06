package frc.robot.autos;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodS;
import frc.robot.subsystems.IndexerS;
import frc.robot.subsystems.IntakePivotS;
import frc.robot.subsystems.IntakeRollerS;
import frc.robot.subsystems.SpindexerS;
import frc.robot.subsystems.TurretS;
import frc.robot.subsystems.IndexerS.IndexerConstants;
import frc.robot.util.AutoAlign;
import frc.robot.util.POI;
import frc.robot.util.TriggerCommand;
import frc.robot.util.TriggerUtil;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.*; // Static import for WPILib Commands

public class AutoCommands {
        // You need these dependencies passed in
        private final CommandSwerveDrivetrain m_drivebase;
        private final Autos autos; // Reference to your Autos class
        private final HoodS m_hood;
        private final IntakePivotS m_intakePivot;
        private final IntakeRollerS m_intakeRoller;
        private final TurretS m_turret;
        private final IndexerS m_indexer;
        private final SpindexerS m_Spindexer;

        SwerveRequest m_intakeDriveRequest = new SwerveRequest.ApplyRobotSpeeds()
                        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity)
                        .withSpeeds(new ChassisSpeeds(1.6, 0.0, 0));

        public AutoCommands(CommandSwerveDrivetrain drivebase, Autos autos, HoodS hood, IntakePivotS intakePivot,
                        IntakeRollerS intakeRoller, TurretS turret,IndexerS indexer, SpindexerS spindexer) {
                this.m_drivebase = drivebase;
                this.autos = autos;
                this.m_hood = hood;
                this.m_intakePivot = intakePivot;
                this.m_intakeRoller = intakeRoller;
                this.m_turret = turret;
                this.m_indexer = indexer;
                this.m_Spindexer = spindexer;
        }

        /**
         * Creates a routine that intakes from the center line
         * 
         * @param helpPose             Pose to go through on the way to the final pose
         *                             to intake
         * @param helpPoseEntryAngle
         * @param helpPoseTolerance    Minimum radius for robot to be in from helpPose
         *                             that triggers the next command
         * @param intakePose           Pose to go through before slowDriveForward
         * @param intakePoseEntryAngle
         * @param intakePoseTolerance  Mnimum radius for robot to be in from intakePose
         *                             that triggers the next command
         * @param driveTime            Time to drive forward collecting fuel
         * @return command that intakes from the center line
         */
        public Command autoToIntake(
                        Pose2d helpPose,
                        Rotation2d helpPoseEntryAngle,
                        Distance helpPoseTolerance,
                        Pose2d intakePose,
                        Rotation2d intakePoseEntryAngle,
                        Distance intakePoseTolerance,
                        Time driveTime) {

                return Commands.sequence(
                                // TO DO: ADD HOOD CLAMPING REQUIREMENT, INTAKE ROLLERS
                                new AutoAlign(helpPose, helpPoseEntryAngle, m_drivebase).until(
                                                TriggerUtil.isWithinRadius(
                                                                () -> helpPose.getTranslation(),
                                                                () -> m_drivebase.state.Pose,
                                                                () -> helpPoseTolerance)),
                                new AutoAlign(intakePose, intakePoseEntryAngle, m_drivebase).until(
                                                TriggerUtil.isWithinRadius(
                                                                () -> intakePose.getTranslation(),
                                                                () -> m_drivebase.state.Pose,
                                                                () -> intakePoseTolerance)),

                                (m_drivebase.applyRequest(() -> m_intakeDriveRequest)
                                                .withTimeout(driveTime)));

        }

        /**
         * Creates a command that intakes from the center line
         * 
         * @param helpPose             Pose to go through on the way to the final pose
         *                             for scoring
         * @param helpPoseEntryAngle
         * @param helpPoseTolerance    Minimum radius for robot to be in from helpPose
         *                             that triggers the next command
         * @param targetpose           Final pose to drive to for scoring
         * @param targetPoseEntryAngle
         * @return command that intakes from the center line
         */
        public Command autoBackFromIntake(Pose2d helpPose,
                        Rotation2d helpPoseEntryAngle,
                        Distance helpPoseTolerance,
                        Pose2d targetpose,
                        Rotation2d targetPoseEntryAngle) {
                // TO DO: ADD HOOD UNCLAMPING REQUIREMENT
                return Commands.sequence(
                                new AutoAlign(helpPose, helpPoseEntryAngle, m_drivebase).until(
                                                TriggerUtil.isWithinRadius(
                                                                () -> helpPose.getTranslation(),
                                                                () -> m_drivebase.state.Pose,
                                                                () -> helpPoseTolerance)),
                                new AutoAlign(targetpose, targetPoseEntryAngle, m_drivebase));

        }

        public Command fuelIntake() {
                return Commands.parallel(
                                m_intakePivot.setAngle(() -> IntakePivotS.IntakePivotConstants.kCWLimit),
                                m_intakeRoller.setVoltage(() -> IntakeRollerS.rollerConstants.kIntakeVoltage));
        }
        //auto hood angle command
        public Command Score() {
                return Commands.parallel(
                                m_hood.autoHoodAngle(),
                                m_indexer.setVoltage(() -> IndexerConstants.kIntakeVoltage),
                                m_Spindexer.setVelocity(()-> SpindexerS.SpindexerConstants.kVelocity));
        }
}
