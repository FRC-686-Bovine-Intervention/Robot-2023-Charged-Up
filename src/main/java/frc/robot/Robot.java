// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.framework.SubsystemController;
import frc.robot.subsystems.vision.Vision;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Log.Exclude
public class Robot extends LoggedRobot {
  private SubsystemController subsystemController;

  @Override
  public void robotInit() {
    setUseTiming(isReal()); // Run as fast as possible during replay
    //LoggedNetworkTables.getInstance().addTable("/SmartDashboard"); // Log & replay "SmartDashboard" values (no tables are logged by default).
    Logger.getInstance().recordMetadata("ProjectName", "BullWhip"); // Set a metadata value

    if (isReal()) {
      Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1"));
      Logger.getInstance().addDataReceiver(new NT4Publisher());
    } else {
        String logPath = LogFileUtil.findReplayLog();
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath));
        Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }

    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    subsystemController = SubsystemController.getInstance();
    subsystemController.register(Vision.getInstance());
    subsystemController.register(Drive.getInstance());
    subsystemController.start();
    io.github.oblarg.oblog.Logger.configureLoggingAndConfig(this, false);
  }
  @Override
  public void robotPeriodic() {
    subsystemController.run();
    io.github.oblarg.oblog.Logger.updateEntries();
  }
  @Override
  public void autonomousInit() {
  }
  @Override
  public void autonomousPeriodic() {
  }
  @Override
  public void teleopInit() {}
  @Override
  public void teleopPeriodic() {
    DriverInteraction.getInstance().run();
  }
  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {}
  @Override
  public void testInit() {}
  @Override
  public void testPeriodic() {}
  @Override
  public void simulationInit() {}
  @Override
  public void simulationPeriodic() {}
}
