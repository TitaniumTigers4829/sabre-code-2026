// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.subsystems.pose.Pose;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveAlign {

  public static SwerveDrive swerve;
  // Rotation PID for auto-aligning
  public static final PIDController rotationPID = new PIDController(0.0099, 0.0, .0);
  public static final double rotationKS = 0.018;
  public static final double rotationMax = 0.40;
  public static final double rotationIZone = 2.0; // degrees

  static {
    rotationPID.enableContinuousInput(-180.0, 180.0);
    rotationPID.setIZone(rotationIZone); // Only use Integral term within this range
    rotationPID.setIntegratorRange(rotationKS * -2, rotationKS * 2);
    rotationPID.reset();
  }

  public static void errorReset() {
    errorReset(rotationPID);
  }

  public static void errorReset(PIDController pid) {
    pid.reset();
  }

  public static void setTarget(Rotation2d targetAngle) {
    setTarget(targetAngle, rotationPID);
  }

  public static void setTarget(Rotation2d targetAngle, PIDController pid) {
    pid.setSetpoint(targetAngle.getDegrees());
    SmartDashboard.putNumber("SwerveAlign/Target Angle", targetAngle.getDegrees());
  }

  public static double getSpeed() {
    return getSpeed(swerve.getEstimatedPose().getRotation());
  }

  public static double getSpeed(Rotation2d currentAngle) {
    return getSpeed(currentAngle, rotationPID);
  }

  // TODO Unify with PID Swerve logic
  public static double getSpeed(Rotation2d currentAngle, PIDController pid) {
    double currentDeg = currentAngle.getDegrees();
    double correction = pid.calculate(currentDeg);
    double feedForward = rotationKS * Math.signum(correction);
    double output = MathUtil.clamp(correction + feedForward, -rotationMax, rotationMax);

    SmartDashboard.putNumber("SwerveAlign/Current Angle", currentDeg);
    SmartDashboard.putNumber("SwerveAlign/PID correction", correction);
    SmartDashboard.putNumber("SwerveAlign/Feedforward", feedForward);
    SmartDashboard.putNumber("SwerveAlign/Output", output);

    return output;
  }

  public static boolean aligned() {
    return aligned(rotationPID);
  }

  public static boolean aligned(double tolerance) {
    return aligned(rotationPID, tolerance);
  }

  public static boolean aligned(PIDController pid) {
    return pid.atSetpoint();
  }

  public static boolean aligned(PIDController pid, double tolerance) {
    return Math.abs(pid.getError()) < tolerance;
  }
}
