// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  TalonFX m_intakeAngleMotor;
  TalonFX m_intakeAngleEncoder;
  VoltageConfigs m_VoltageConfigs;
  PIDController m_pidController = new PIDController(.2, 0.01, 0.001);
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_intakeAngleEncoder = new TalonFX(0);
    m_intakeAngleMotor = new TalonFX(0);
    CurrentLimitsConfigs m_currentLimitConfigs = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(40)
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimit(50)
    .withSupplyCurrentLimitEnable(true);

    m_targetPos = targetPos();
  }
  

  
  private double targetPos() {
    return -1 * (m_intakeAngleMotor.getPosition().getValueAsDouble() - getPos()) / (6.578128) * 360;
  }

  public double getPos() {
    return m_intakeAngleEncoder.getPosition().getValueAsDouble();
  }

  public void resetEncoderPos() {
    m_intakeAngleEncoder.setPosition(0);
  }

  public double setTargetPos(double target) {
    m_targetPos = target;
  }

  
  @Override
  public void periodic() {

  if (pidMode) {
     double fb = m_pidController.calculate(getPos(), );
  }
    // This method will be called once per scheduler run
  }
}
