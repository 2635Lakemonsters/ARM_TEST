// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  Joystick m_rightJoystick;
 
  public ArmSubsystem(Joystick leftJoystick, Joystick rightJoystick) {
    m_rightJoystick = rightJoystick;
    m_leftJoystick = leftJoystick;
   // m_intakeAngleEncoder = new TalonFX();
    m_intakeAngleMotor = new TalonFX(11);
    CurrentLimitsConfigs m_currentLimitConfigs = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(150)
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
  
  public double convertThrottleToPosition(Joystick joystick){
    // Positon = -30 * Throttle + 30
    double position = -30*joystick.getThrottle()+30;
    return position;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm angle", getPosDegrees());
    SmartDashboard.putNumber("raw rotations", getPos());
    SmartDashboard.putNumber("arm angle temp", m_intakeAngleMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("arm stator amps", m_intakeAngleMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("arm supply amps", m_intakeAngleMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("arm motor volts", m_intakeAngleMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("left throttle", m_leftJoystick.getThrottle());
    // NetworkTableInstance default = NetworkTableInstance.getDefault();
    //   default.getRoot().putBoolean("running", true);
    //   default.getNested("arm").putDouble("arm angle", getPosDegrees());
    //   default.getNested("motors").putDouble("motor2", 1200.0);
    //   default.getNested("sensors").putDouble("odometryX", 5.2);
    //   default.getNested("sensors").putDouble("odometryY", 3.1);

    
    if (pidMode) {
      double ff = -2.6;
      m_targetPos = convertThrottleToPosition(m_rightJoystick);
      double fb = m_pidController.calculate(getPosDegrees(), m_targetPos);
      SmartDashboard.putNumber("right thottle", m_rightJoystick.getThrottle());
      SmartDashboard.putNumber("target pos", m_targetPos);
      SmartDashboard.putNumber("feedback", fb);
      setVolts(
        // m_leftJoystick.getThrottle() * 
        ff * Math.cos(getPosDegrees() / 180 * Math.PI)
        -fb*2
      );
    }
    // This method will be called once per scheduler run
  }
}
