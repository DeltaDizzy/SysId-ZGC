// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
//import frc.robot.SysId.Logging.SysIdDrivetrainLogger;
import frc.robot.SysId.Logging.SysIdGeneralMechanismLogger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private SysIdDrivetrainLogger sysid = new SysIdDrivetrainLogger();
  private SysIdGeneralMechanismLogger sysid = new SysIdGeneralMechanismLogger();
  //private Drive drive = new Drive();
  private Arm arm = new Arm();
  private Notifier sysidLoop = new Notifier(this::sysidRun);

  @Override
  public void autonomousInit() {
    sysid.initLogging();
    sysidLoop.startPeriodic(0.005);
  }

  private void sysidRun() {
    /*sysid.log(
      sysid.measureVoltage(drive.getLeftMotors()),
      sysid.measureVoltage(drive.getRightMotors()), 
      drive.getLeftDistanceRotations(), 
      drive.getRightDistanceRotations(), 
      drive.getLeftVelocityRps(), 
      drive.getRightVelocityRps(), 
      drive.getGyroAngle().getDegrees(), 
      drive.getGyroRate()
    );
    sysid.setMotorControllers(-sysid.getLeftMotorVoltage(), drive.getLeftMotors());
    sysid.setMotorControllers(sysid.getRightMotorVoltage(), drive.getRightMotors());
    drive.diffDrive.feed();*/
    
    
    sysid.log(
      sysid.measureVoltage(List.of(arm.yAxisMotor)), 
      arm.getPosition(), 
      arm.getVelocity()
    );
    sysid.setMotorControllers(sysid.getMotorVoltage(), List.of(arm.yAxisMotor));
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    sysidLoop.stop();
    //sysid.setMotorControllers(0, drive.getLeftMotors());
    //sysid.setMotorControllers(0, drive.getRightMotors());
    sysid.setMotorControllers(0, List.of(arm.yAxisMotor));
    sysid.sendData();
  }
}
