package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int INTAKE_MOTOR_1_ID = 9;
  public static final int INTAKE_MOTOR_2_ID = 0; // ORIGINALLY 17
  public static final int PIVOT_MOTOR_RIGHT_ID = 23; // 41, ALSO ACTUALLY LEFT
  // public static final int PIVOT_MOTOR_LEFT_ID = 23;
  public static final int CANCODER_ID = 19;

  public static final double INTAKE_P = 1.0;
  public static final double INTAKE_I = 0.0;
  public static final double INTAKE_D = 0.0;
  public static final double INTAKE_S = 0.0; // I fear this idek
  public static final double INTAKE_V = 0.0; // IK this is bad but we will see how it goes
  public static final double INTAKE_A = 0.0; // Suicidal thoughts are coming to me

  public static final double PIVOT_P = 25.0;
  public static final double PIVOT_I = 0.0; // I having so much fun :D
  public static final double PIVOT_D = 0.1; // 0.5; // Something happened to my brain
  public static final double PIVOT_S =
      0.0; // 1.5; // NO NO I DON'T WANT TO TAKE SPANISH WITH MS. WALSH
  public static final double PIVOT_V =
      0.0; // 1.5; // GOD FOR LOVE OF ALL WHY SHE KIDNAPS ME IN SPANISH CLASS
  public static final double PIVOT_A = 0.0; // THIS IS WORST THAN 9/11
  public static final double PIVOT_G = 0.0; // 0.8;

  public static final double ZERO_ANGLE = 0.427; // 0.09;

  // DONT TOUCH THESE ARE SOMEWHAT GOOD CURRENT LIMIT VALUES
  public static final double OUTER_STATOR_CURRENT_LIMIT = 80;
  public static final double OUTER_SUPPLY_CURRENT_LIMIT = 65;

  public static final double INNER_STATOR_CURRENT_LIMIT = 80;
  public static final double INNER_SUPPLY_CURRENT_LIMIT = 5;

  public static final double PIVOT_STATOR_CURRENT_LIMIT = 80;
  public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 25;

  public static final double PIVOT_DOWN_POSITION = 0.8;
  public static final double PIVOT_UP_POSITION = 0.12; // -0.33
  public static final double PIVOT_ALL_THE_WAY_UP_POSITION = 0.027;

  public static final double PIVOT_TICS = 50;
  public static final double PIVOT_BOUNCE_LIMIT = 25;

  public static final double PIVOT_BOUNCE_LOWER_POSITION = 0.5;
  public static final double PIVOT_BOUNCE_HIGHER_POSITION = 0.3;

  public static final double MAX_ANGLE = 1;
  public static final double ACCEPTABLE_RANGE = 0.3;
  public static final double MIN_ANGLE = 0.0; // I HATE THIS SO MUCH
  public static final double INTAKE_SPEED_OUTER = -1;
  public static final double INTAKE_SPEED_INNER = 1;

  public static final double GEAR_RATIO = 1;
}
