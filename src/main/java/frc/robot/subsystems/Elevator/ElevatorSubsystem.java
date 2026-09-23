// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  public TalonFX leaderMotor = new TalonFX(ElevatorConstants.LEADER_MOTOR_ID);
  public TalonFX followerMotor = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID);

  public TalonFXConfiguration configs = new TalonFXConfiguration();
  public StatusSignal<Angle> position = leaderMotor.getPosition();
  public StatusSignal<Voltage> voltage = leaderMotor.getMotorVoltage();

  // Remember this to continue, this bum ass honduras struggle with a get call
  // free candys are at andriianasa van

  public ElevatorSubsystem() {
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 0.0;
    configs.CurrentLimits.SupplyCurrentLimit = 0.0;
    configs.CurrentLimits.SupplyCurrentLimitEnable = false;
    
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

    configs.Slot0.kP = 0.0;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kS = 0.0;
    configs.Slot0.kV = 0.0;
    configs.Slot0.kA = 0.0;
    configs.Slot0.kG = 0.0;
    configs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    leaderMotor.getConfigurator().apply(configs);
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    followerMotor.getConfigurator().apply(configs);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
