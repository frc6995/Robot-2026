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

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {
        private final Autopilot kAutopilot;

        private final APTarget m_target;
        private final CommandSwerveDrivetrain m_drivetrain;
        private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
                        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                        .withDriveRequestType(DriveRequestType.Velocity)
                        .withHeadingPID(4, 0, 0); // Replace with constants later

        private SwerveDriveState swerveState = new SwerveDriveState();

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
