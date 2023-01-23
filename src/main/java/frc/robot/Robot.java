// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.SysId.Logging.SysIdDrivetrainLogger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private SysIdDrivetrainLogger sysid = new SysIdDrivetrainLogger();
  private Drive drive = new Drive();
  private Notifier sysidLoop = new Notifier(this::sysidRun);

  @Override
  public void autonomousInit() {
    sysid.initLogging();
    sysidLoop.startPeriodic(0.005);
  }

  private void sysidRun() {
    sysid.log(
      sysid.measureVoltage(drive.getLeftMotors()),
      sysid.measureVoltage(drive.getRightMotors()), 
      drive.getLeftDistanceMeters(), 
      drive.getRightDistanceMeters(), 
      drive.getWheelSpeeds().leftMetersPerSecond, 
      drive.getWheelSpeeds().rightMetersPerSecond, 
      drive.getGyroAngle().getDegrees(), 
      drive.getGyroRate()
    );
    sysid.setMotorControllers(sysid.getLeftMotorVoltage(), drive.getLeftMotors());
    sysid.setMotorControllers(sysid.getRightMotorVoltage(), drive.getRightMotors());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    sysidLoop.stop();
    sysid.setMotorControllers(0, drive.getLeftMotors());
    sysid.setMotorControllers(0, drive.getRightMotors());
    sysid.sendData();
  }
}
