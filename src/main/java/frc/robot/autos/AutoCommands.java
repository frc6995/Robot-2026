package frc.robot.autos;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodS;
import frc.robot.subsystems.HoodS.hoodConstants;
import frc.robot.util.AutoAlign;
import frc.robot.util.POI;

import static edu.wpi.first.wpilibj2.command.Commands.*; // Static import for WPILib Commands

public class AutoCommands {
    // You need these dependencies passed in
    private final CommandSwerveDrivetrain m_drivebase;
    private final Autos autos; // Reference to your Autos class
    private final HoodS m_hood;

    SwerveRequest mIntakeRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSpeeds(new ChassisSpeeds(0.3, 0, 0));

    public AutoCommands(CommandSwerveDrivetrain drivebase, Autos autos, HoodS hood) {
        this.m_drivebase = drivebase;
        this.autos = autos;
        this.m_hood = hood;
    }

    public Command autoIntake(BooleanSupplier isLeftSide, Pose2d intakePose, double driveTime) {
       if (isLeftSide.getAsBoolean()) {
            return Commands.sequence(
                    m_hood.setAngle(hoodConstants.kStowAngle), // replace with threshold command
                    new AutoAlign(POI.get(), POI.TRL1Entry.get(), m_drivebase));
        } else {
            return Commands.sequence(
                    m_hood.setAngle(hoodConstants.kStowAngle), // replace with threshold command
                    new AutoAlign(POI.RR1.get(), POI.TRR1Entry.get(), m_drivebase));
        }



        }
    
    
    public Command autoBackFromIntake(BooleanSupplier isLeftSide) {
        if (isLeftSide.getAsBoolean()) {
            return Commands.sequence(
                    m_hood.setAngle(hoodConstants.kStowAngle), // replace with threshold command
                    new AutoAlign(POI.TRL1.get(), POI.TRL1Entry.get(), m_drivebase));
        } else {
            return Commands.sequence(
                    m_hood.setAngle(hoodConstants.kStowAngle), // replace with threshold command
                    new AutoAlign(POI.T.get(), POI.TRR1Entry.get(), m_drivebase));
        }

    }

    public Command fuelIntake() {
        return Commands.parallel(
                m_drivebase.runRequest(mIntakeRequest).withTimeout(2.0)
        );
    }
}
