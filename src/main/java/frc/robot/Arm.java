// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  WPI_TalonFX motor;
  /** Creates a new Arm. */
  public Arm() {
    motor = new WPI_TalonFX(7);
    motor.setSelectedSensorPosition(0);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public List<WPI_TalonFX> getMotor() {
    return List.of(motor);
  }

  public double getPosition() {
    return sensorUnitsToMotorRotations(motor.getSelectedSensorPosition());
  }

  private double sensorUnitsToMotorRotations(double units) {
    // convert from units to motor rotations
    return units / 2048.0;
  }

  public double getVelocity() {
    return sensorVelocityToArmSpeed(motor.getSelectedSensorVelocity());
  }

  private double sensorVelocityToArmSpeed(double sensorVelocity) {
    // input is sensor units per 100ms
    // per second is more so multiply
    double perSecond = sensorVelocity * 10;
    //convert sensors units per second to arm rotations per second
    return sensorUnitsToMotorRotations(perSecond);
  }
}
