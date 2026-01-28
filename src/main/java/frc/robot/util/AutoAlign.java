package frc.robot.util;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import com.therekrab.autopilot.Autopilot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.Autos.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {

        public static class AutoAlignConstants {
                private static final double DEFAULT_ACCELERATION = 15;
                private static final double DEFAULT_JERK = 12;

                public static APConstraints DEFAULT_CONSTRAINTS = new APConstraints(DEFAULT_ACCELERATION, DEFAULT_JERK);
        }

        private final Autopilot kAutopilot;

        private final APTarget m_target;
        private final CommandSwerveDrivetrain m_drivetrain;
        private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
                        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                        .withDriveRequestType(DriveRequestType.Velocity)
                        .withHeadingPID(4, 0, 0); // Replace with constants later

        private SwerveDriveState swerveState = new SwerveDriveState();

        /**
         * Uses default constraints, beeline path
         * 
         * @param targetPose Pose2d to align to
         * @param drivetrain Drivetrain subsystem
         */
        public AutoAlign(Pose2d targetPose, CommandSwerveDrivetrain drivetrain) {
                this(new APTarget(targetPose), drivetrain, AutoAlignConstants.DEFAULT_CONSTRAINTS);
        }

        /**
         * Uses default constraints, path respects entry angle
         * 
         * @param targetPose Pose2d to align to
         * @param entryAngle Entry angle to modify approach
         * @param drivetrain Drivetrain subsystem
         */
        public AutoAlign(Pose2d targetPose, Rotation2d entryAngle, CommandSwerveDrivetrain drivetrain) {
                this(new APTarget(targetPose).withEntryAngle(entryAngle), drivetrain,
                                AutoAlignConstants.DEFAULT_CONSTRAINTS);
        }

        /**
         * Auto allign constructor needing all parameters
         * 
         * @param targetPose  Pose2d to align to
         * @param constraints Entry angle to modify approach
         * @param drivetrain  Drivetrain subsystem
         */
        public AutoAlign(APTarget target, CommandSwerveDrivetrain drivetrain, APConstraints constraints) {
                this.m_target = target;
                this.m_drivetrain = drivetrain;

                APProfile kProfile = new APProfile(constraints)
                                .withErrorXY(Centimeters.of(2))
                                .withErrorTheta(Degrees.of(0.5))
                                .withBeelineRadius(Centimeters.of(8));

                kAutopilot = new Autopilot(kProfile);

                addRequirements(drivetrain);
        }

        @Override
        public void execute() {
                swerveState = m_drivetrain.getState();
                APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_target);

                m_drivetrain.setControl(m_request
                                .withVelocityX(out.vx())
                                .withVelocityY(out.vy())
                                .withTargetDirection(out.targetAngle()));
        }

        @Override
        public boolean isFinished() {
                return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_target);
        }

}
