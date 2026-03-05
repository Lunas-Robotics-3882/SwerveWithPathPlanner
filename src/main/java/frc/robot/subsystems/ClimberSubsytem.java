// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//CTRE Imports Motor Imports
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsytem extends SubsystemBase {


  private double position;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private ShuffleboardTab tab = Shuffleboard.getTab("Climber");
  private GenericEntry climberEncoder = tab.add("Climber Encoder", 0).getEntry();
  private GenericEntry climberVelocity = tab.add("Climber Velocity", 0).getEntry();
  /** Creates a new ClimberSubsystem. */


  private final TalonFX climber = new TalonFX(13, "LUNACAN");
  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 1 */
  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  //Encoder Values
  double homePosition = 0;
  double UpPos = 129;
  double ClimbPosition = 80;

  public ClimberSubsytem() {
    position = 0;
  
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    configs.Slot1.kP = 80; // An error of 1 rotation results in 60 A output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
    // Peak output of 120 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
      .withPeakReverseTorqueCurrent(Amps.of(-120));

      climber.getConfigurator().apply(configs);
      climber.setNeutralMode(NeutralModeValue.Brake);
      climber.setPosition(position);
  }

public void setHoldPosition(double holdposition) {
  position = holdposition;
}

public void setVelocity(double speed)
{
  climber.setControl(m_velocityVoltage.withVelocity(speed));
  position = climber.getPosition().getValueAsDouble();
}

public boolean CheckPositionHome()
{
 return MathUtil.isNear(homePosition,climber.getPosition().getValueAsDouble(), 1);
}

public boolean CheckUpPost()
{
 return MathUtil.isNear(UpPos,climber.getPosition().getValueAsDouble(), 1);
}

public void setPosition(double setPoint)
{
  climber.setControl(m_positionTorque.withPosition(setPoint));
}

public double getEncoder()
{
  return climber.getPosition().getValueAsDouble();
}

public Command slowUp()
{
  return run(() -> this.setVelocity(20));
}


public Command slowDown()
{
  return run(() -> this.setVelocity(-20));
}

public void stop()
{
  climber.setControl(m_brake);
}

public Command newStop() {

  return run (() -> this.setPosition(climber.getPosition().getValueAsDouble()));
}

public Command stopCommand()
{
  return run(() -> this.stop());
}


public Command withPosition(double setPoint)
{
  return run(() -> this.setPosition(setPoint));
}

public Command setHomePosition()
{
  return run(() -> this.setPosition(homePosition)); 
}


public Command setUpPosition()
{
  return run(() -> this.setPosition(UpPos)); 
}

public Command setClimbPosition()
{
  return run (() -> this.setPosition(ClimbPosition));
}


@Override
public void periodic() {
//climber.setControl(m_positionTorque.withPosition(position));
//SmartDashboard.putBoolean("limit checks", LimitChecks());
SmartDashboard.putNumber("Climber Encoder", climber.getPosition().getValueAsDouble());
SmartDashboard.putNumber("Climber Velocity", climber.getVelocity().getValueAsDouble());

}
}