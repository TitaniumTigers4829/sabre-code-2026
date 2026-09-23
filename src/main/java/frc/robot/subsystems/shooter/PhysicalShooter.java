// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.extras.logging.LoggedTunableNumber;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;

/** Add your docs here. */
public class PhysicalShooter implements ShooterInterface {

  private boolean isUpToSpeed = false;
  private int pauseRollerFloorCounter = 0;
  private boolean isPausingRollerFloor =
      true; // we start with the roller floor paused to let any balls stuck in the injector have a
  // chance to be shot first
  private boolean rollerWasUpToSpeed = false;
  boolean reachedSpeedOnce = false;

  LoggedTunableNumber flywheelRPS = new LoggedTunableNumber("Shooter/RPS", 0.0);

  private final TalonFX topLeftFlywheelMotor =
      new TalonFX(ShooterConstants.TOP_LEFT_FLYWHEEL_MOTOR_ID);
  private final TalonFX bottomLeftFlywheelMotor =
      new TalonFX(ShooterConstants.BOTTOM_LEFT_FLYWHEEL_MOTOR_ID);
  private final TalonFX topRightFlywheelMotor =
      new TalonFX(ShooterConstants.TOP_RIGHT_FLYWHEEL_MOTOR_ID);
  // private final TalonFX bottomRightFlywheelMotor =
  // new TalonFX(ShooterConstants.BOTTOM_RIGHT_FLYWHEEL_MOTOR_ID);
  private final TalonFX rollerMotor = new TalonFX(ShooterConstants.ROLLER_MOTOR_ID);
  private final TalonFX rollerFloor = new TalonFX(ShooterConstants.ROLLER_MOTOR_2_ID);

  // MotorAlignmentValue motorAlignment = MotorAlignmentValue.Opposed;

  private final SingleLinearInterpolator flywheelRPMLookupValues;

  private final VelocityTorqueCurrentFOC rpsRequest = new VelocityTorqueCurrentFOC(0.0);
  // private final DutyCycleOut dutyCyleOut = new DutyCycleOut(0.0);
  // private final VelocityVoltage rpsRequest = new VelocityVoltage(0.0);

  // private final MotionMagicTorqueCurrentFOC mmTorqueRequest = new
  // MotionMagicTorqueCurrentFOC(0.0);
  // private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);

  private final TalonFXConfiguration leaderFlywheelConfig = new TalonFXConfiguration();

  private final StatusSignal<AngularVelocity> currentRPS;
  private final StatusSignal<AngularVelocity> rollerVelocity;

  private final LinearFilter rollerVelocityFilter = LinearFilter.movingAverage(10);

  private int counter1 = 0;

  // private final TalonFXConfiguration followerFlywheelConfig = new TalonFXConfiguration();

  public PhysicalShooter() {

    leaderFlywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leaderFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderFlywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;
    leaderFlywheelConfig.Slot0.kP = ShooterConstants.FLYWHEEL_P;
    leaderFlywheelConfig.Slot0.kI = ShooterConstants.FLYWHEEL_I;
    leaderFlywheelConfig.Slot0.kD = ShooterConstants.FLYWHEEL_D;
    leaderFlywheelConfig.Slot0.kS = ShooterConstants.FLYWHEEL_S;
    leaderFlywheelConfig.Slot0.kV = ShooterConstants.FLYWHEEL_V;
    leaderFlywheelConfig.Slot0.kA = ShooterConstants.FLYWHEEL_A;
    // TODO: drop down
    leaderFlywheelConfig.CurrentLimits.StatorCurrentLimit = 120;

    // leaderFlywheelConfig.CurrentLimits.SupplyCurrentLimit = 160;
    leaderFlywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
    leaderFlywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;

    leaderFlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // leaderFlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // TODO: timeouts
    // TODO: unscuff
    bottomLeftFlywheelMotor.getConfigurator().apply(leaderFlywheelConfig);
    topRightFlywheelMotor.getConfigurator().apply(leaderFlywheelConfig);

    leaderFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    topLeftFlywheelMotor.getConfigurator().apply(leaderFlywheelConfig);
    // bottomRightFlywheelMotor.getConfigurator().apply(leaderFlywheelConfig);
    // kickerMotor.getConfigurator().apply(leaderFlywheelConfig);
    // leaderFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // spindexerMotor.inve
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    // TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
    // TODO: tune
    rollerConfig.CurrentLimits.StatorCurrentLimit = 60;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 10;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rollerMotor.getConfigurator().apply(rollerConfig);
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerFloor.getConfigurator().apply(rollerConfig);
    // followerFlywheelMotor.setControl(
    //     new Follower(leaderFlywheelMotor.getDeviceID(), motorAlignment));

    flywheelRPMLookupValues =
        new SingleLinearInterpolator(ShooterConstants.DISTANCE_TO_FLYWHEEL_RPM);

    currentRPS = topRightFlywheelMotor.getVelocity();
    rollerVelocity = rollerMotor.getVelocity();

    currentRPS.setUpdateFrequency(100);
    rollerVelocity.setUpdateFrequency(50);

    ParentDevice.optimizeBusUtilizationForAll(
        topLeftFlywheelMotor, topRightFlywheelMotor, bottomLeftFlywheelMotor);
  }

  public void updateInputs(ShooterInputs inputs) {
    BaseStatusSignal.refreshAll(currentRPS);

    inputs.flywheelRPS = currentRPS.getValueAsDouble();
  }

  public double getFlywheelRPS() {
    currentRPS.refresh();
    return currentRPS.getValueAsDouble();
  }

  // test
  public void setPercentOutput(double distance, boolean useOneMotor) {
    // double desiredSpeed = flywheelRPMLookupValues.getLookupValue(distance);
    double desiredSpeed = flywheelRPS.get();
    topLeftFlywheelMotor.setControl(rpsRequest.withVelocity(desiredSpeed));
    topRightFlywheelMotor.setControl(rpsRequest.withVelocity(desiredSpeed));
    bottomLeftFlywheelMotor.setControl(rpsRequest.withVelocity(desiredSpeed));
    // bottomRightFlywheelMotor.setControl(rpsRequest.withVelocity(desiredSpeed));
    this.isUpToSpeed =
        Math.abs(desiredSpeed - currentRPS.refresh().getValueAsDouble())
            < ShooterConstants.FLYWHEEL_ERROR_TOLERANCE;
    SmartDashboard.putNumber("desiredRPS", desiredSpeed);
    SmartDashboard.putNumber("currentRPS", currentRPS.refresh().getValueAsDouble());
    SmartDashboard.putBoolean("ready to shoot", isUpToSpeed());

    if (isUpToSpeed()) {
      reachedSpeedOnce = true;
    }

    if (counter1 < 20) {
      setRollerSpeed(-ShooterConstants.SPINDEXER_SHOOT_SPEED);
      // setKickerSpeed(-ShooterConstants.KICKER_PERCENT_OUTPUT);
      return;
    }

    // setKickerSpeed(ShooterConstants.KICKER_PERCENT_OUTPUT);

    if (counter1 > 20) {
      setRollerSpeed(ShooterConstants.SPINDEXER_SHOOT_SPEED);
      // setKickerSpeed(ShooterConstants.KICKER_PERCENT_OUTPUT);
      return;
    }

    double averageRollerVelocity =
        Math.abs(rollerVelocityFilter.calculate(rollerVelocity.refresh().getValueAsDouble()));

    if (averageRollerVelocity > 2.5) rollerWasUpToSpeed = true;

    if (reachedSpeedOnce) {
      if (isPausingRollerFloor) pauseRollerFloorCounter++;

      if (averageRollerVelocity < 2.5 && rollerWasUpToSpeed) {
        isPausingRollerFloor = true;
        rollerWasUpToSpeed = false;
      }

      double rollerSpeed = ShooterConstants.SPINDEXER_SHOOT_SPEED;

      if (isPausingRollerFloor && pauseRollerFloorCounter < 55) {
        rollerSpeed = 0;
      } else if (isPausingRollerFloor && pauseRollerFloorCounter >= 55) {
        isPausingRollerFloor = false;
        rollerWasUpToSpeed = false;
        pauseRollerFloorCounter = 0;
      }

      setRollerSpeed(rollerSpeed);
      setKickerSpeed(ShooterConstants.KICKER_PERCENT_OUTPUT);
    } else {
      // setRollerSpeed(0.0);
      // setKickerSpeed(0.0);
    }
  }

  public boolean isUpToSpeed() {
    return this.isUpToSpeed;
  }

  // UNUSED CUZ JACK IS A CHUD
  public void passFuel() {
    topRightFlywheelMotor.set(-0.5);
    topLeftFlywheelMotor.set(-0.5);
    // bottomRightFlywheelMotor.setControl(rpsRequest.withVelocity(50));
    bottomLeftFlywheelMotor.set(-0.5);
    rollerFloor.set(0.5);
    this.isUpToSpeed =
        Math.abs(50 - currentRPS.refresh().getValueAsDouble())
            < ShooterConstants.FLYWHEEL_ERROR_TOLERANCE;
    // SmartDashboard.putNumber("desiredRPS", desiredSpeed);
    // SmartDashboard.putNumber("currentRPS", currentRPS.refresh().getValueAsDouble());
    setRollerSpeed(ShooterConstants.SPINDEXER_SHOOT_SPEED);

    SmartDashboard.putNumber("desired rps", 50);
    SmartDashboard.putBoolean("ready to shoot", isUpToSpeed());

    if (isUpToSpeed()) {
      reachedSpeedOnce = true;
    }

    double averageRollerVelocity =
        Math.abs(rollerVelocityFilter.calculate(rollerVelocity.refresh().getValueAsDouble()));

    if (averageRollerVelocity > 2.5) rollerWasUpToSpeed = true;

    if (reachedSpeedOnce) {
      if (isPausingRollerFloor) pauseRollerFloorCounter++;

      if (averageRollerVelocity < 2.5 && rollerWasUpToSpeed) {
        isPausingRollerFloor = true;
        rollerWasUpToSpeed = false;
      }

      double rollerSpeed = ShooterConstants.SPINDEXER_SHOOT_SPEED;

      if (isPausingRollerFloor && pauseRollerFloorCounter < 50) {
        rollerSpeed = 0;
      } else if (isPausingRollerFloor && pauseRollerFloorCounter >= 50) {
        isPausingRollerFloor = false;
        rollerWasUpToSpeed = false;
        pauseRollerFloorCounter = 0;
      }

      setRollerSpeed(rollerSpeed);
      // setKickerSpeed(ShooterConstants.KICKER_PERCENT_OUTPUT);
    } else {
      setRollerSpeed(0.0);
      // setKickerSpeed(0.0);
    }
  }

  public void setSpeed(double rps) {
    topRightFlywheelMotor.setControl(rpsRequest.withVelocity(rps));
    topLeftFlywheelMotor.setControl(rpsRequest.withVelocity(rps));
    // bottomRightFlywheelMotor.setControl(rpsRequest.withVelocity(rps));
    bottomLeftFlywheelMotor.setControl(rpsRequest.withVelocity(rps));
  }

  public void stopShoot() {
    topLeftFlywheelMotor.set(0);
    bottomLeftFlywheelMotor.set(0);
    topRightFlywheelMotor.set(0);
    // bottomRightFlywheelMotor.set(0);
    rollerMotor.set(0);
    rollerFloor.set(0);
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  // public void setKickerSpeed(double speed) {
  //   kickerAndRollerMotor.set(speed);
  // }

  public boolean setIsAimingProperly(boolean isAimingProperly) {
    return isAimingProperly;
  }

  public void startingShoot() {
    isUpToSpeed = false;
    pauseRollerFloorCounter = 0;
    isPausingRollerFloor = true;
    rollerWasUpToSpeed = false;
    reachedSpeedOnce = false;
    counter1 = 0;
  }
}
