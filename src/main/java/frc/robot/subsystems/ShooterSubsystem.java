package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.Timer;

// Shooter negative
public class ShooterSubsystem extends SubsystemBase {

  private final CANBus canbus = new CANBus();
  private final TalonFX m_shooter = new TalonFX(60, canbus);
  private final TalonFX m_followershooter = new TalonFX(61, canbus);

  /* Start at velocity 0, use slot 1 */
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();





  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

         
  TalonFXConfiguration configs = new TalonFXConfiguration();
  /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
  configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
  configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
  configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
  configs.Slot0.kI = 0; // No output for integrated error
  configs.Slot0.kD = 0; // No output for error derivative
  // Peak output of 8 volts
  configs.Voltage.withPeakForwardVoltage(8).withPeakReverseVoltage(-8);
  m_shooter.getConfigurator().apply(configs);
  m_shooter.setNeutralMode(NeutralModeValue.Coast);

  m_followershooter.getConfigurator().apply(configs);
  m_followershooter.setControl(new Follower(m_shooter.getDeviceID(),MotorAlignmentValue.Opposed));
  m_followershooter.setNeutralMode(NeutralModeValue.Coast);
  }

//  COMMANDS  //

  public void disable()
  {
    m_shooter.setControl(m_brake);
  }

  public void setVelocity(double desiredRotationsPerSecond)
{
    m_shooter.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
}

  public void shooter()
{
 //m_shooter.setControl(m_velocityVoltage.withVelocity(-10));
  m_shooter.setControl(m_velocityVoltage.withVelocity(-52.5));
}

  public void outtake()
{
    m_shooter.setControl(m_velocityVoltage.withVelocity(30));
}

public void outtakeOtherSpeed()
{
    m_shooter.setControl(m_velocityVoltage.withVelocity(28));
}


public Command shootCommand()
{
  return run(() -> this.shooter());
}

public Command outtakeCommand()
{
  return run(() -> this.outtake());
}

public Command withVelocity(double desiredRotationsPerSecond)
{
  return runOnce(() -> this.setVelocity(desiredRotationsPerSecond));
}

public Command stop()
{
    return run(() -> this.disable());
}


  @Override
public void periodic() {
//This method will be called once per scheduler run
SmartDashboard.putNumber("Shooter Encoder", m_shooter.getPosition().getValueAsDouble());
SmartDashboard.putNumber("Shooter Velocity", m_shooter.getVelocity().getValueAsDouble());
  }
}
