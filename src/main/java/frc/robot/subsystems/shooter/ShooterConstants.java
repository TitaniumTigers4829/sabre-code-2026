// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public class ShooterConstants {

  public static int TOP_LEFT_FLYWHEEL_MOTOR_ID = 61;
  public static int BOTTOM_LEFT_FLYWHEEL_MOTOR_ID = 57;
  public static int TOP_RIGHT_FLYWHEEL_MOTOR_ID = 9;
  public static int BOTTOM_RIGHT_FLYWHEEL_MOTOR_ID = 0;
  public static int ROLLER_MOTOR_ID = 27;
  public static int ROLLER_MOTOR_2_ID = 22;

  public static double SHOOTER_HEIGHT_FROM_GROUND = 0;
  public static double GEAR_RATIO = 0.95;

  public static double FLYWHEEL_S = 0; // 0.2;
  public static double FLYWHEEL_V = 0; // 0.111;
  public static double FLYWHEEL_A = 0;
  public static double FLYWHEEL_P = 48294829; // 20
  public static double FLYWHEEL_I = 0;
  public static double FLYWHEEL_D = 0;

  // 5 rpm
  public static double FLYWHEEL_ERROR_TOLERANCE = 2;

  public static double PASS_SHOOTER_SPEED = -1;
  public static double KICKER_PERCENT_OUTPUT = 1;
  public static double SPINDEXER_INTAKE_SPEED = -1; // 0.675
  public static double SPINDEXER_SHOOT_SPEED = -0.675; // 0.675

  // Lookup table for rpms needed for certain distances
  public static double[][] DISTANCE_TO_FLYWHEEL_RPM = {
    // Distance from hub in meters, needed rps of flywheel
    {1.5, 47.29},
    {2.0, 50},
    {2.5, 54},
    {3.0, 57},
    {3.5, 61},
    {4.0, 63.4829},
    {4.5, 64.4829},
    {5.0, 66.4829},
    {5.5, 66.4829},
    // FOR PASSING ONLY - GUESSED VALUES:
    {6, 68},
    {13, 120}
  };
}
