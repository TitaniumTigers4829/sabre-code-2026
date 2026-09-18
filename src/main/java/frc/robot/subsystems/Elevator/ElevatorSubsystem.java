// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  public TalonFX leaderMotor = new TalonFX(ElevatorConstants.LEADER_MOTOR_ID);
  public TalonFX followerMotor = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID);

  public TalonFXConfiguration configs = new TalonFXConfiguration();
  public StatusSignal position = leaderMotor.getPosition();
  public StatusSignal voltage = leaderMotor.getMotorVoltage();
  
  //Remember this to continue, this bum ass honduras struggle with a get call

  

  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
