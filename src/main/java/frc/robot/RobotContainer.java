// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.sim.CANdleSimState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.AutoCommands;
import frc.robot.autos.Autos;
import frc.robot.generated.ChoreoVars;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.FlyWheelS;
import frc.robot.subsystems.flywheel.NoneFlyWheelS;
import frc.robot.subsystems.flywheel.RealFlyWheelS;
import frc.robot.subsystems.hood.HoodS;
import frc.robot.subsystems.hood.NoneHoodS;
import frc.robot.subsystems.hood.RealHoodS;
import frc.robot.subsystems.hood.RealHoodS.HoodConstants;
import frc.robot.subsystems.indexer.IndexerS;
import frc.robot.subsystems.indexer.NoneIndexerS;
import frc.robot.subsystems.indexer.RealIndexerS;
import frc.robot.subsystems.intakepivot.IntakePivotS;
import frc.robot.subsystems.intakepivot.NoneIntakePivotS;
import frc.robot.subsystems.intakepivot.RealIntakePivotS;
import frc.robot.subsystems.intakepivot.RealIntakePivotS.IntakePivotConstants;
import frc.robot.subsystems.intakeroller.IntakeRollerS;
import frc.robot.subsystems.intakeroller.NoneIntakeRollerS;
import frc.robot.subsystems.intakeroller.RealIntakeRollerS;
import frc.robot.subsystems.spindexer.NoneSpindexerS;
import frc.robot.subsystems.spindexer.RealSpindexerS;
import frc.robot.subsystems.spindexer.SpindexerS;
import frc.robot.subsystems.turret.NoneTurretS;
import frc.robot.subsystems.turret.RealTurretS;
import frc.robot.subsystems.turret.TurretS;
import frc.robot.subsystems.climb.climbextension.NoneClimbExtensionS;
import frc.robot.subsystems.climb.climbextension.RealClimbExtensionS;
import frc.robot.subsystems.climb.climbextension.ClimbExtensionS;
import frc.robot.subsystems.vision.detection.NoneODVision;
import frc.robot.subsystems.vision.detection.ObjectVision;
import frc.robot.subsystems.vision.detection.RealODVision;
import frc.robot.subsystems.vision.detection.SimODVision;
import frc.robot.util.AutoAlign;
import frc.robot.util.AutoAlignFixedHeading;
import frc.robot.util.POI;
import frc.robot.util.ShooterController;
import frc.robot.util.Telemetry;
import frc.robot.util.ClimbConstants.ClimbExtensionConstantsRecord;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class RobotContainer {
    public static final TelemetryVerbosity kTelemetryVerbosity = TelemetryVerbosity.LOW;

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

    private final Telemetry logger = new Telemetry();

    public static final CommandXboxController joystick = new CommandXboxController(0);

    @Logged
    public final CommandSwerveDrivetrain m_drivetrain = new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);

    @Logged(name = "Flywheel")
    private final FlyWheelS m_flywheel = new RealFlyWheelS();
    @Logged(name = "Hood")
    private final HoodS m_hood = new RealHoodS(() -> m_drivetrain.state, () -> m_drivetrain.lastState);
    // private final HoodS m_hood = new NoneHoodS();

    @Logged(name = "Indexer")
    private final IndexerS m_indexer = new RealIndexerS();
    @Logged(name = "IntakePivot")
    private final IntakePivotS m_intakePivot = new NoneIntakePivotS();
    @Logged(name = "IntakeRoller")
    private final IntakeRollerS m_intakeRoller = new RealIntakeRollerS();
    @Logged(name = "Spindexer")
    private final SpindexerS m_spindexer = new RealSpindexerS();
    @Logged(name = "Turret")
    private final TurretS m_turret = new RealTurretS(() -> m_drivetrain.state.Pose, () -> m_drivetrain.state.Speeds,
            () -> m_intakePivot.isIntakeDeployed());
//     private final TurretS m_turret = new NoneTurretS();
    @Logged(name = "ClimbExtension")
    private final ClimbExtensionS m_climbExtension = new NoneClimbExtensionS();

    // @Logged(name = "ObjectDetection")
    private final ObjectVision m_objectVision = new NoneODVision(() -> m_drivetrain.state.Pose);

    private final AutoCommands m_autoCommands = new AutoCommands(m_drivetrain, null, m_hood, m_intakePivot,
            m_intakeRoller, m_turret, m_indexer, m_spindexer, m_flywheel, m_climbExtension, m_objectVision);

    private final AutoFactory autoFactory;
    private final Autos autoRoutines;
    public final AutoChooser m_chooser = new AutoChooser();

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    public RobotContainer() {
        ShooterController.initialize(() -> m_drivetrain.state, () -> m_drivetrain.lastState,
                (pose) -> ShooterController.getAimLocation(pose));

        autoFactory = m_drivetrain.createAutoFactory();
        autoRoutines = new Autos(m_autoCommands, m_drivetrain, autoFactory, this, m_hood, m_intakePivot, m_intakeRoller,
                m_turret,
                m_indexer, m_spindexer, m_flywheel, m_objectVision);
        SmartDashboard.putData("Auto Mode", m_chooser);
        setDefaultCommands();
        configureBindings();
    }

    public void periodic() {
        m_objectVision.update();
    }

    public double xButtonPressedTime = 0;
    public boolean intakeState = false;
    public Angle hoodTarget = HoodConstants.kLowerLimit;

    private void setDefaultCommands() {
        m_intakePivot.setDefaultCommand(
                m_intakePivot.setAngle(() -> IntakePivotConstants.kStowAngle));

        m_intakeRoller.setDefaultCommand(
                m_intakeRoller.setVoltage(Volts.of(0)));

        m_turret.setDefaultCommand(
                m_turret.runSOTF(ShooterController.getInstance()::getCachedData));

        m_hood.setDefaultCommand(m_hood.setAngle(() -> Degrees.zero()));

        m_flywheel.setDefaultCommand(
                m_flywheel.runSOTF(ShooterController.getInstance()::getCachedData));
        m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                m_drivetrain.applyRequest(
                        () -> {
                            var xSpeed = MathUtil.applyDeadband(-joystick.getLeftY(), 0.1) * 4.2;
                            var ySpeed = MathUtil.applyDeadband(-joystick.getLeftX(), 0.1) * 4.2;
                            var rotationSpeed = MathUtil.applyDeadband(-joystick.getRightX(), 0.1) * 2 * Math.PI;
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
                                            rotationSpeed); // Drive counterclockwise with negative X (left)
                        }));

    }

    private void configureBindings() {
        // Left stick: Movement
        // Right stick: Rotation
        // D-pad: Robot relative translation
        // A: Intake toggle
        // B: Cardinal direction hold
        // X: Climb
        // RT: Hold to Shoot
        // Start: Current home turret (enabled)
        // Back: Home all (enabled); Set positions (disabled)

        // robot relative driving with D-pad
        joystick.povCenter().whileFalse(driveIntakeRelativePOV());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // A intake toggle
        joystick.a().toggleOnTrue(m_autoCommands.fuelIntake());

        // B button align to cardinal direction
        joystick.b().whileTrue(
                m_drivetrain.applyRequest(
                        () -> {
                            var xSpeed = -joystick.getLeftY() * 4.2;
                            var ySpeed = -joystick.getLeftX() * 4.2;
                            var rotSpeed = m_drivetrain.calculateThetaPID(
                                    AutoAlignFixedHeading.cardinalizeHeadingNS(
                                            m_drivetrain.state.Pose
                                                    .getRotation()));

                            return m_driveRequest
                                    .withVelocityX(
                                            xSpeed) // Drive forward with
                                                    // negative Y (forward)
                                    .withVelocityY(
                                            ySpeed) // Drive left with
                                                    // negative X (left)
                                    .withRotationalRate(
                                            rotSpeed);
                        } // Drive counterclockwise with negative X (left)
                ));

        // RT: hold to shoot
        joystick.rightTrigger().whileTrue(m_autoCommands.Score());

        // Start: Current home turret (enabled)
        joystick.start().onTrue(m_turret.driveToHome());


         joystick.y().onTrue(m_hood.setAngle(()->Degrees.of(30)));

        // Back: Home all, set all positions in disable
        joystick.back()
                .onTrue(Commands.parallel(
                        m_turret.resetEncoder(),
                        m_intakePivot.resetEncoder(),
                        m_hood.resetEncoder()));
       

        joystick.rightTrigger().whileTrue(m_autoCommands.Score());
        joystick.leftTrigger()
                .whileTrue(m_autoCommands.intakeWiggle(Degrees.of(30), IntakePivotConstants.kLowerLimit, 0.4));
        joystick.leftTrigger().onFalse(m_intakePivot.setAngle(() -> IntakePivotConstants.kLowerLimit));

        m_drivetrain.registerTelemetry(logger::telemeterize);
        // JS: Why is this a deferred proxy?
        RobotModeTriggers.autonomous()
                .onTrue(m_turret.driveToHome().andThen(
                        m_turret.aimAtFieldPose(() -> POI.HUB1.get().getTranslation(), () -> m_drivetrain.state.Pose))
                        .until(() -> !DriverStation.isAutonomous()));
    }

    private RobotCentric m_robotCentricRequest = new RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    public Command driveIntakeRelativePOV() {
        return m_drivetrain.applyRequest(() -> {
            double pov = Units.degreesToRadians(-joystick.getHID().getPOV());
            double adjustSpeed = Units.feetToMeters(3); // m/s
            return m_robotCentricRequest.withVelocityX(
                    Math.cos(pov) * adjustSpeed).withVelocityY(
                            Math.sin(pov) * adjustSpeed)
                    .withRotationalRate(
                            -joystick.getRightX() * 2
                                    * Math.PI);
        });
    }

    public Command getAutonomousCommand() {
        return m_chooser.selectedCommand();

    }

}
