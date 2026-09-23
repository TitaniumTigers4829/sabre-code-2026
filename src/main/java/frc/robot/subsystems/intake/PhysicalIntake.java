package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HardwareConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class PhysicalIntake implements IntakeInterface {
  private TalonFX intakeMotorOuter = new TalonFX(IntakeConstants.INTAKE_MOTOR_1_ID);
  private TalonFX intakeMotorInside = new TalonFX(IntakeConstants.INTAKE_MOTOR_2_ID);
  private TalonFX intakePivotMotorRight =
      new TalonFX(IntakeConstants.PIVOT_MOTOR_RIGHT_ID, HardwareConstants.RIO_CAN_BUS_STRING);
  // private TalonFX intakePivotMotorLeft = new TalonFX(IntakeConstants.PIVOT_MOTOR_LEFT_ID);

  private final CANcoder pivotEncoder =
      new CANcoder(IntakeConstants.CANCODER_ID, HardwareConstants.RIO_CAN_BUS_STRING);

  private MotionMagicVoltage request = new MotionMagicVoltage(0.0);
  // private MotorAlignmentValue pivotMotorAlignment = MotorAlignmentValue.Opposed;
  private TalonFXConfiguration intakeOuterConfig;
  private TalonFXConfiguration intakeInnerConfig;
  private TalonFXConfiguration pivotConfig;
  private CANcoderConfiguration encoderConfig;

  public StatusSignal<Angle> intakeAngle;
  public StatusSignal<AngularVelocity> intakePivotSpeed;

  public PhysicalIntake() {
    intakeOuterConfig = new TalonFXConfiguration();
    intakeInnerConfig = new TalonFXConfiguration();
    pivotConfig = new TalonFXConfiguration();
    encoderConfig = new CANcoderConfiguration();

    encoderConfig.MagnetSensor.MagnetOffset = -IntakeConstants.ZERO_ANGLE;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    pivotEncoder.getConfigurator().apply(encoderConfig, HardwareConstants.TIMEOUT_SECONDS);

    intakeOuterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeOuterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeOuterConfig.Slot0.kP = IntakeConstants.INTAKE_P;
    intakeOuterConfig.Slot0.kI = IntakeConstants.INTAKE_I;
    intakeOuterConfig.Slot0.kD = IntakeConstants.INTAKE_D;
    intakeOuterConfig.Slot0.kS = IntakeConstants.INTAKE_S;
    intakeOuterConfig.Slot0.kV = IntakeConstants.INTAKE_V;
    intakeOuterConfig.Slot0.kA = IntakeConstants.INTAKE_A;
    intakeOuterConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.OUTER_STATOR_CURRENT_LIMIT;
    intakeOuterConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.OUTER_SUPPLY_CURRENT_LIMIT;
    intakeOuterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeOuterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeInnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeInnerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeInnerConfig.Slot0.kP = IntakeConstants.INTAKE_P;
    intakeInnerConfig.Slot0.kI = IntakeConstants.INTAKE_I;
    intakeInnerConfig.Slot0.kD = IntakeConstants.INTAKE_D;
    intakeInnerConfig.Slot0.kS = IntakeConstants.INTAKE_S;
    intakeInnerConfig.Slot0.kV = IntakeConstants.INTAKE_V;
    intakeInnerConfig.Slot0.kA = IntakeConstants.INTAKE_A;
    intakeInnerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.INNER_STATOR_CURRENT_LIMIT;
    intakeInnerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INNER_SUPPLY_CURRENT_LIMIT;
    intakeInnerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeInnerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.MotorOutput.DutyCycleNeutralDeadband = 0.00001;
    pivotConfig.Slot0.kP = IntakeConstants.PIVOT_P;
    pivotConfig.Slot0.kI = IntakeConstants.PIVOT_I;
    pivotConfig.Slot0.kD = IntakeConstants.PIVOT_D;
    pivotConfig.Slot0.kS = IntakeConstants.PIVOT_S;
    pivotConfig.Slot0.kV = IntakeConstants.PIVOT_V;
    pivotConfig.Slot0.kA = IntakeConstants.PIVOT_A;
    pivotConfig.Slot0.kG = IntakeConstants.PIVOT_G;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;
    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.OUTER_STATOR_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.OUTER_SUPPLY_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.PIVOT_DOWN_POSITION;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.MIN_ANGLE;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    pivotConfig.MotionMagic.MotionMagicAcceleration = 5.0;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 2.5;

    intakeMotorOuter.getConfigurator().apply(intakeOuterConfig);
    intakeMotorInside.getConfigurator().apply(intakeInnerConfig);
    intakePivotMotorRight.getConfigurator().apply(pivotConfig);

    // pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // intakePivotMotorLeft.getConfigurator().apply(pivotConfig);

    intakeAngle = pivotEncoder.getPosition();
    intakePivotSpeed = intakePivotMotorRight.getVelocity();

    // pivotEncoder.setPosition(pivotEncoder.getAbsolutePosition().getValueAsDouble());
    pivotEncoder.setPosition(0);
    // intakePivotMotorRight.setPosition(0.0);
    // intakePivotMotorLeft.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, intakeAngle, intakePivotSpeed);
    ParentDevice.optimizeBusUtilizationForAll(
        intakeMotorOuter, intakeMotorInside, intakePivotMotorRight); // intakePivotMotorLeft
  }

  public void updateInputs(IntakeInputs inputs) {
    intakeAngle.refresh();
    intakePivotSpeed.refresh();

    inputs.intakeAngle = intakeAngle.getValueAsDouble();
    inputs.intakePivotSpeed = intakePivotSpeed.getValueAsDouble();
    inputs.isIntakeDeployed = isIntakeDeployed();
    SmartDashboard.putNumber("intake angle", inputs.intakeAngle);
    // SmartDashboard.putNumber(
    //     "real intake position", intakePivotMotorRight.getPosition().getValueAsDouble());
    // System.out.println("WORKING");
  }

  public void setIntakeAngle(double angle) {
    intakePivotMotorRight.setControl(request.withPosition(angle));
    // intakePivotMotorLeft.setControl(request.withPosition(angle));
  }

  public void intakeFuel() {
    intakeMotorOuter.set(IntakeConstants.INTAKE_SPEED_OUTER);
    intakeMotorInside.set(IntakeConstants.INTAKE_SPEED_INNER);
  }

  public void outakeFuel() {
    intakeMotorOuter.set(-IntakeConstants.INTAKE_SPEED_OUTER);
    intakeMotorInside.set(-IntakeConstants.INTAKE_SPEED_INNER);
  }

  public void setSpeed(double speed) {
    intakeMotorOuter.set(speed);
    intakeMotorInside.set(speed);
  }

  public void setPivotSpeed(double speed) {
    intakePivotMotorRight.set(speed);
    // intakePivotMotorLeft.set(speed);
  }

  public void setAngle(double angle) {
    intakePivotMotorRight.setControl(request.withPosition(angle));
    // intakePivotMotorLeft.setControl(
    //     new Follower(intakePivotMotorLeft.getDeviceID(), pivotMotorAlignment));
  }

  public void setPivotSpeedUp() {
    intakePivotMotorRight.setControl(request.withPosition(IntakeConstants.PIVOT_UP_POSITION));
    // intakePivotMotorLeft.setControl(request.withPosition(IntakeConstants.PIVOT_UP_POSITION));
  }

  public void setPivotSpeedDown() {
    intakePivotMotorRight.setControl(request.withPosition(IntakeConstants.PIVOT_DOWN_POSITION));
    // intakePivotMotorLeft.setControl(request.withPosition(IntakeConstants.PIVOT_DOWN_POSITION));
  }

  public double getIntakeAngle() {
    intakeAngle.refresh();
    return intakeAngle.getValueAsDouble();
  }

  public void zeroAngle() {
    pivotEncoder.setPosition(0);
  }

  public double getIntakeSpeed() {
    intakePivotSpeed.refresh();
    return intakePivotSpeed.getValueAsDouble();
  }

  @AutoLogOutput(key = "isintake")
  public boolean isIntakeDeployed() {
    return Math.abs(getIntakeAngle() - IntakeConstants.PIVOT_DOWN_POSITION)
        < IntakeConstants.ACCEPTABLE_RANGE;
  }
}
