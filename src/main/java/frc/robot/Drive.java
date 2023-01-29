// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
//#region imports
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//#endregion
public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  //#region declarations
  WPI_TalonFX Left_Front = new WPI_TalonFX(1);
  WPI_TalonFX Right_Front = new WPI_TalonFX(3);
  WPI_TalonFX Left_Back = new WPI_TalonFX(2);
  WPI_TalonFX Right_Back = new WPI_TalonFX(4);
  MotorControllerGroup LeftMCG = new MotorControllerGroup(Left_Front, Left_Back);
  MotorControllerGroup RightMCG = new MotorControllerGroup(Right_Front, Right_Back);
  DifferentialDrive diffDrive = new DifferentialDrive(LeftMCG, RightMCG);

  AHRS gyro = new AHRS();

  //Simulation Stuff
  //#endregion
  
  
  public Drive() {
    Left_Front.configFactoryDefault();
    Right_Front.configFactoryDefault();
    Left_Back.configFactoryDefault();
    Right_Back.configFactoryDefault();

    Left_Back.follow(Left_Front);
    Right_Back.follow(Right_Front);

    Left_Back.setInverted(InvertType.FollowMaster);
    Right_Back.setInverted(InvertType.FollowMaster);

    LeftMCG.setInverted(true);
  }

  /**
   * gets the distance that the right side of the robot traveled in meters 
   * @return
   * the distance the right side of the robot has traveled
   */
  public double getLeftDistanceMeters() {
    return Left_Front.getSelectedSensorPosition() * 8.991487388626803e-06;
  }
  /**
   * gets the distance that the right side of the robot traveled in meters 
   * @return
   * the distance the right side of the robot has traveled
   */
  public double getRightDistanceMeters() {
    return Right_Front.getSelectedSensorPosition() * 8.991487388626803e-06;
  }

  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  //#region SysId Getters
  public List<WPI_TalonFX> getLeftMotors() {
    return List.of(Left_Front, Left_Back);
  }

  public List<WPI_TalonFX> getRightMotors() {
    return List.of(Right_Front, Right_Back);
  }

  public double getGyroRate() {
    return gyro.getRate();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // TODO: add converter for wheel speeds
    return new DifferentialDriveWheelSpeeds(Left_Front.getSelectedSensorVelocity(), Right_Front.getSelectedSensorVelocity());
  }
  //#endregion

  @Override
  public void periodic() {

  }

public double getLeftDistanceRotations() {
  // divide by 2048 for shaft rots
  return Left_Front.getSelectedSensorPosition() / 2048;
}

public double getRightDistanceRotations() {
  return Right_Front.getSelectedSensorPosition() / 2048;
}

public double getLeftVelocityRps() {
  double raw = Left_Front.getSelectedSensorVelocity();
  // ticks per sec = ticks per 100ms * 10
  double perSec = raw * 10;
  // 2048 ticks = 1 rot
  return perSec / 2048.0;
}

public double getRightVelocityRps() {
  double raw = Right_Front.getSelectedSensorVelocity();
  // ticks per sec = ticks per 100ms * 10
  double perSec = raw * 10;
  // 2048 ticks = 1 rot
  return perSec / 2048.0;
}
  
}