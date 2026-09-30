// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterInterface extends Subsystem {
  @AutoLog
  public static class ShooterInputs { // For values
    public double flywheelRPS = 0.0;
    public double flywheelMotorVoltage = 0.0;
    public double flywheelDutyCycle = 0.0;
    public double flywheelDesiredRPS = 0.0;
    public double flywheelStatorCurrent = 0.0;
    public double flywheelVelocity = 0.0;
    public double flywheelRPMError = 0.0;
    public double flywheelMotorTemp = 0.0;
  }

  public default void updateInputs(ShooterInputs inputs) {}

  public default double getFlywheelRPS() {
    return 0.0;
  }

  public default void setSpeed(double targetRPM) {}

  public default void setVolts(double volts) {}

  public default void setRollerSpeed(double speed) {}

  public default void setPercentOutput(double distance, boolean useOneMotor) {}

  public default void setPercentOutput2(double speed) {}

  public default double getVolts() {
    return 0.0;
  }

  public default void passFuel() {}

  public default void stopShoot() {}

  public default void setStaticSpeed(double speedRps) {}

  public default void openLoop(double output) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}

  public default void setKickerSpeed(double speed) {}

  public default void startingShoot() {}

  public default boolean isReadyToShoot() {
    return false;
  }

  public default boolean setIsAimingProperly(boolean isAimingProperly) {
    return false;
  }
}
