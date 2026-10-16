// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public class ShooterConstants {

  public static int TOP_LEFT_FLYWHEEL_MOTOR_ID = 61;
  public static int BOTTOM_LEFT_FLYWHEEL_MOTOR_ID = 57;
  public static int TOP_RIGHT_FLYWHEEL_MOTOR_ID = 34;
  public static int BOTTOM_RIGHT_FLYWHEEL_MOTOR_ID = 47;
  public static int ROLLER_MOTOR_ID = 27;
  public static int ROLLER_MOTOR_2_ID = 17;
  public static int RIGHT_ROLLER_MOTOR_ID = 6;

  public static double SHOOTER_HEIGHT_FROM_GROUND = 0;
  public static double GEAR_RATIO = 1;

  public static double FLYWHEEL_S = 0; // 0.2;
  public static double FLYWHEEL_V = 0; // 0.111;
  public static double FLYWHEEL_A = 0;
  public static double FLYWHEEL_P = 48294829; // 20
  public static double FLYWHEEL_I = 0;
  public static double FLYWHEEL_D = 0;

  // 5 rpm
  public static double FLYWHEEL_ERROR_TOLERANCE = 2;

  public static double PASS_SHOOTER_SPEED = -70;
  public static double KICKER_PERCENT_OUTPUT = 70;
  public static double SPINDEXER_INTAKE_SPEED = -70; // 0.675
  public static double SPINDEXER_SHOOT_SPEED = -55; // 0.675
  public static double FLYWHEEL_STATIC_SPEED = 60;

  // Lookup table for rpms needed for certain distances
  public static double[][] DISTANCE_TO_FLYWHEEL_RPM = {
    // Distance from hub in meters, needed rps of flywheel
    {1.5, 35.29},
    {2.0, 40},
    {2.5, 50},
    {3.0, 55},
    {3.5, 57},
    {4.0, 58.4829},
    {4.5, 60.4829},
    {5.0, 62.4829},
    {5.5, 62.4829},
    // FOR PASSING ONLY - GUESSED VALUES:
    {6, 68},
    {13, 100}
  };
}
