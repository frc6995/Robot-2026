// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

@Logged
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    @Logged(name = "RobotContainer")
    private final RobotContainer m_robotContainer;
    private double autoSimTime = 20;

    public Robot() {

        Epilogue.configure(config -> {
            // Log only to disk, instead of the default NetworkTables logging
            // Note that this means data cannot be analyzed in realtime by a dashboard
            // config.backend = new FileBackend(DataLogManager.getLog());

            if (isSimulation()) {
                // If running in simulation, then we'd want to re-throw any errors that
                // occur so we can debug and fix them!
                config.errorHandler = ErrorHandler.crashOnError();
            }

            // Change the root data path
            config.root = "Telemetry";

            // minimum importance level that will be logged. DEBUG is lowest level and will
            // log everything by default
            config.minimumImportance = Logged.Importance.DEBUG;
        });
        Epilogue.bind(this);

        m_robotContainer = new RobotContainer();
    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    m_robotContainer.periodic();
    /*if (enableLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      //double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    LimelightHelpers.SetRobotOrientation(kLimelightName, headingDeg, 0.0, 0.0, 0.0, 0.0, 0.0);
    //Periodically retrieve the results from the limelight and extract the pose.
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelightName);
    if (limelightMeasurement.tagCount > 0) {
      m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      m_robotContainer.drivetrain.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
    }*/

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (RobotBase.isSimulation()) {
      Commands.waitSeconds(autoSimTime)
          .andThen(
              () -> {
                DriverStationSim.setEnabled(false);
                DriverStationSim.notifyNewData();
              })
          .onlyWhile(DriverStation::isAutonomousEnabled)
          .schedule();
    }
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

      // schedule the autonomous command (example)
      if (m_autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(m_autonomousCommand);
      }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
