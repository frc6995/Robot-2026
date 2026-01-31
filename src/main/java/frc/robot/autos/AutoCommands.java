package frc.robot.autos;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
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

    public AutoCommands(CommandSwerveDrivetrain drivebase, Autos autos, HoodS hood) {
        this.m_drivebase = drivebase;
        this.autos = autos;
        this.m_hood = hood;
    }

    SwerveRequest mIntakeRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity)
            .withSpeeds(new ChassisSpeeds(0.3, 0, 0));

    public Command autoBackFromIntake(BooleanSupplier isLeftSide) {
        if (isLeftSide.getAsBoolean()) {
            return Commands.sequence(
                m_hood.setAngle(hoodConstants.kStowAngle),
                new AutoAlign(POI.TRL1.get(),  POI.testEntry.get(), m_drivebase)
            );
        } else {
            return sequence(
                autos.getTrajectoryCommand("AutoBackFromIntakeRight", m_drivebase),
                new AutoAlign(m_drivebase, false)
            );
        }

{
}