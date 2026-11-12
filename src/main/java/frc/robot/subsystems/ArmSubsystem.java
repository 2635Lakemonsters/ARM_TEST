// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  TalonFX m_intakeAngleMotor;
  //TalonFX m_intakeAngleEncoder;
  double m_targetPos;
  double m_initialPos;
  boolean pidMode = true;
  VoltageConfigs m_VoltageConfigs;
  PIDController m_pidController = new PIDController(.02, 0.001, 0.001);
  Joystick m_leftJoystick;
 
  public ArmSubsystem(Joystick leftJoystick) {
    m_leftJoystick = leftJoystick;
   // m_intakeAngleEncoder = new TalonFX();
    m_intakeAngleMotor = new TalonFX(11);
    CurrentLimitsConfigs m_currentLimitConfigs = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(40)
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimit(50)
    .withSupplyCurrentLimitEnable(true);
  
    m_initialPos = m_intakeAngleMotor.getPosition().getValueAsDouble();
    //m_targetPos = targetPos();
    m_intakeAngleMotor.getConfigurator().apply(m_currentLimitConfigs);
  }
  
  public void changePidMode(boolean newMode) {
    pidMode = newMode;
  }

  private double getPosDegrees() {
    return -1 * (
      m_intakeAngleMotor.getPosition().getValueAsDouble()- m_initialPos
    ) / (9) * 360;  //turns it into degrees, 3 is gear ratio
  }

  public double getPos() {
    return m_intakeAngleMotor.getPosition().getValueAsDouble();
  }

  public void resetEncoderPos() {
    m_intakeAngleMotor.setPosition(0);
  }
  // public double setTargetPos(double target) {
  //   return m_targetPos = target;
  // }

  public void setVolts(double volts) {
    m_intakeAngleMotor.setVoltage(volts);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm angle", getPosDegrees());
    SmartDashboard.putNumber("raw rotations", getPos());
    SmartDashboard.putNumber("arm angle temp", m_intakeAngleMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("arm stator current", m_intakeAngleMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("arm supply current", m_intakeAngleMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("arm motor voltage", m_intakeAngleMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("left joystick throttle", m_leftJoystick.getThrottle());
    
    if (pidMode) {
      double ff = 2.6;
      double fb = m_pidController.calculate(getPos(), m_targetPos );
      setVolts(m_leftJoystick.getThrottle() * ff * Math.cos(getPosDegrees() / 180 * Math.PI));
    }
    // This method will be called once per scheduler run
  }
}
