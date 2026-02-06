// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import frc.robot.autos.AutoCommands;
import frc.robot.autos.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlyWheelS;
import frc.robot.subsystems.HoodS;
import frc.robot.subsystems.IndexerS;
import frc.robot.subsystems.IntakePivotS;
import frc.robot.subsystems.IntakeRollerS;
import frc.robot.subsystems.SpindexerS;
import frc.robot.subsystems.TurretS;
import frc.robot.subsystems.IntakePivotS.IntakePivotConstants;
import frc.robot.subsystems.IntakeRollerS.IntakeRollerConstants;
import frc.robot.util.AutoAlignFixedHeading;
import frc.robot.util.Telemetry;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                  // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                      // second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                     // motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    private final FlyWheelS m_flywheel = new FlyWheelS();
    private final HoodS m_hood = new HoodS(() -> m_drivetrain.state.Pose, () -> m_drivetrain.state.Speeds);
    private final IndexerS m_indexer = new IndexerS();
    private final IntakePivotS m_intakepivot = new IntakePivotS();
    private final IntakeRollerS m_intakeroller = new IntakeRollerS();
    private final SpindexerS m_spindexer = new SpindexerS();
    private final TurretS m_turret = new TurretS();
    private final AutoCommands m_AutoCommands = new AutoCommands(m_drivetrain, null, m_hood, m_intakepivot, m_intakeroller, m_turret, m_indexer, m_spindexer, m_flywheel);

    private final AutoFactory autoFactory;
    private Mechanism2d VISUALIZER;
    private final Autos autoRoutines;
    public final AutoChooser m_chooser = new AutoChooser();

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    public RobotContainer() {

        m_drivetrain.resetOdometry(new Pose2d());
        VISUALIZER = logger.MECH_VISUALIZER;

        SmartDashboard.putData("Visualzer", VISUALIZER);

        autoFactory = m_drivetrain.createAutoFactory();
        autoRoutines = new Autos(m_drivetrain, autoFactory, this, m_hood, m_intakepivot, m_intakeroller, m_turret, m_indexer,m_spindexer,m_flywheel);
        SmartDashboard.putData("Auto Mode", m_chooser);
        configureBindings();

    }

    public double xButtonPressedTime = 0;
    public boolean intakeState = false;

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                m_drivetrain.applyRequest(
                        () -> {
                            var xSpeed = -joystick.getLeftY() * 4.2;
                            var ySpeed = -joystick.getLeftX() * 4.2;
                            var rotationSpeed = -joystick.getRightX() * 2 * Math.PI;

                            if (DriverStation.isAutonomous()) {
                                return m_driveRequest.withVelocityX(0).withVelocityY(0)
                                        .withRotationalRate(0);
                            }
                            return m_driveRequest
                                    .withVelocityX(
                                            xSpeed) // Drive forward with
                                                    // negative Y (forward)
                                    .withVelocityY(
                                            ySpeed) // Drive left with
                                                    // negative X (left)
                                    .withRotationalRate(
                                            rotationSpeed);
                        } // Drive counterclockwise with negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        
        joystick.a().onTrue(new SequentialCommandGroup(
                Commands.runOnce(() -> intakeState = !intakeState),
                m_intakepivot.setAngle(() -> intakeState ? IntakePivotConstants.kFuelIntakeAngle : IntakePivotConstants.kStowAngle),
                m_intakeroller.setVoltage(() -> intakeState ? IntakeRollerConstants.kIntakeVoltage : Volts.of(0))
        ));

        joystick.b().whileTrue(
                m_drivetrain.applyRequest(
                        () -> {
                            var xSpeed = -joystick.getLeftY() * 4.2;
                            var ySpeed = -joystick.getLeftX() * 4.2;
                            var rotSpeed = m_drivetrain.calculateThetaPID(AutoAlignFixedHeading.cardinalizeHeading(m_drivetrain.state.Pose.getRotation()));

                            return m_driveRequest
                                    .withVelocityX(
                                            xSpeed) // Drive forward with
                                                    // negative Y (forward)
                                    .withVelocityY(
                                            ySpeed) // Drive left with
                                                    // negative X (left)
                                    .withRotationalRate(
                                        rotSpeed
                                    );
                        } // Drive counterclockwise with negative X (left)
                )
        );

        joystick.rightTrigger().whileTrue(m_AutoCommands.Score());

        //joystick.leftTrigger().whileTrue();

        //joystick.x();
        
        //😢pain
        m_drivetrain.registerTelemetry(logger::telemeterize);

    }

    public Command getAutonomousCommand() {
        return m_chooser.selectedCommand();

    }

}
