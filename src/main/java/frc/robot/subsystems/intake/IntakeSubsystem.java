// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extras.logging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  private IntakeInterface intakeInterface;
  private IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private double intakePos = IntakeConstants.PIVOT_UP_POSITION;

  private static final LoggedTunableNumber IntakeS = new LoggedTunableNumber("Intake/IntakeS");
  private static final LoggedTunableNumber IntakeV = new LoggedTunableNumber("Intake/IntakeV");
  private static final LoggedTunableNumber IntakeA = new LoggedTunableNumber("Intake/IntakeA");
  private static final LoggedTunableNumber IntakeP = new LoggedTunableNumber("Intake/IntakeP");
  private static final LoggedTunableNumber IntakeI = new LoggedTunableNumber("Intake/IntakeI");
  private static final LoggedTunableNumber IntakeD = new LoggedTunableNumber("Intake/IntakeD");

  static {
    switch (Constants.getRobot()) {
      case COMP_ROBOT, DEV_ROBOT -> {
        IntakeS.initDefault(IntakeConstants.INTAKE_S);
        IntakeV.initDefault(IntakeConstants.INTAKE_V);
        IntakeA.initDefault(IntakeConstants.INTAKE_A);
        IntakeP.initDefault(IntakeConstants.INTAKE_P);
        IntakeI.initDefault(IntakeConstants.INTAKE_I);
        IntakeD.initDefault(IntakeConstants.INTAKE_D);
      }
      default -> {}
    }
  }

  /** Creates a new IntakeSubsystem with hardware interface. */
  public IntakeSubsystem(IntakeInterface intakeInterface) {
    this.intakeInterface = intakeInterface;
  }

  public void setIntakeAngle(double angle) {
    this.intakeInterface.setIntakeAngle(angle);
  }

  public void intakeFuel() {
    this.intakeInterface.intakeFuel();
  }

  public void outakeFuel() {
    this.intakeInterface.outakeFuel();
  }

  public void setSpeed(double speed) {
    this.intakeInterface.setSpeed(speed);
  }

  public void zeroAngle() {
    this.intakeInterface.zeroAngle();
  }

  public void setAngle(double angle) {
    this.intakeInterface.setAngle(angle);
  }

  public void setPivotSpeed(double speed) {
    this.intakeInterface.setPivotSpeed(speed);
  }

  public double getIntakeAngle() {
    return this.intakeInterface.getIntakeAngle();
  }

  public double getIntakeSpeed() {
    return this.intakeInterface.getIntakeSpeed();
  }

  public void setPivotSpeedUp() {
    this.intakeInterface.setPivotSpeedUp();
  }

  public void setPivotSpeedDown() {
    this.intakeInterface.setPivotSpeedDown();
  }

  public boolean isIntakeDeployed() {
    return this.intakeInterface.isIntakeDeployed();
  }

  public void updateInputs() {
    this.intakeInterface.updateInputs(inputs);
  }

  public void setIntakePos(double intakePos) {
    // this.intakePos = intakePos;
    this.intakeInterface.setIntakeAngle(intakePos);
  }

  @Override
  public void periodic() {
    updateInputs();
    SmartDashboard.putBoolean("intake deployed", inputs.isIntakeDeployed);
    SmartDashboard.putNumber("intake angle", inputs.intakeAngle);
    Logger.recordOutput("intake angle reallll", inputs.intakeAngle);
    // setIntakeAngle(intakePos);
    // if (isIntakeDeployed()) {
    // intakeFuel();
    // } else setSpeed(0);
  }
}
