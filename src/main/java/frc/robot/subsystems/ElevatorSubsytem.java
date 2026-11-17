// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsytem extends SubsystemBase {
  private TalonFX m_elevatorMotor;
  private double m_targetHeight;
  private double m_initialPos;
  CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs()
  .withStatorCurrentLimit(40)
  .withStatorCurrentLimitEnable(true)
  .withSupplyCurrentLimit(40)
  .withSupplyCurrentLimitEnable(true);
  VoltageConfigs m_voltageConfigs = new VoltageConfigs();
  /** Creates a new ElevatorSubsytem. */
  public ElevatorSubsytem() {
    m_elevatorMotor = new TalonFX(0);
    m_initialPos = m_elevatorMotor.getPosition().getValueAsDouble();
    m_voltageConfigs.PeakForwardVoltage = 12;
    m_voltageConfigs.PeakReverseVoltage = 12;

    m_voltageConfigs.SupplyVoltageTimeConstant = 10;
    m_elevatorMotor.getConfigurator().apply(m_currentLimits);
    m_elevatorMotor.getConfigurator().apply(m_voltageConfigs);
  }

  public double getPosRotations() {
    m_elevatorMotor.getPosition();

    return m_elevatorMotor.getPosition().getValueAsDouble();
    
  }

  public double getPosDegrees() {
    return (getPosRotations() - m_initialPos) / 9 * 360; //turns it into degrees
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
