package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.CompConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.DevConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.RobotConstants;

/** Swerve Constants */
public class SwerveConstants {

  public static final class DriveConstants {
    public static final double X_POS_TRUST = 0.03; // Meters
    public static final double Y_POS_TRUST = 0.03; // Meters
    public static final double ANGLE_TRUST = Units.degreesToRadians(1); // Radians

    // Wheel base and track width are measured by the center of the swerve modules, not the frame of
    // the robot
    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(25);

    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(19);
    public static final double DRIVE_BASE_DIAMETER =
        Math.sqrt(Math.pow(DriveConstants.TRACK_WIDTH, 2) + Math.pow(DriveConstants.WHEEL_BASE, 2));

    public static final Translation2d[] MODULE_TRANSLATIONS =
        new Translation2d[] {
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Front Right
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Rear Left
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Rear Right
        };

    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    public static final int PIGEON_ID = 0;

    // TODO: Add robot name
    public static final class RobotConstants {
      public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 54;
      public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
      public static final int REAR_LEFT_DRIVE_MOTOR_ID = 26;
      public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 1;

      public static final int FRONT_LEFT_TURN_MOTOR_ID = 7;
      public static final int FRONT_RIGHT_TURN_MOTOR_ID = 2;
      public static final int REAR_LEFT_TURN_MOTOR_ID = 5;
      public static final int REAR_RIGHT_TURN_MOTOR_ID = 6;

      public static final int FRONT_LEFT_CANCODER_ID = 16;
      public static final int FRONT_RIGHT_CANCODER_ID = 13;
      public static final int REAR_LEFT_CANCODER_ID = 10;
      public static final int REAR_RIGHT_CANCODER_ID = 12;

      public static final double FRONT_LEFT_ZERO_ANGLE = 0.134765625;
      public static final double FRONT_RIGHT_ZERO_ANGLE = -0.01220703125;
      public static final double REAR_LEFT_ZERO_ANGLE = 0.02099609375;
      public static final double REAR_RIGHT_ZERO_ANGLE = 0.4111328125;

      public static final SensorDirectionValue FRONT_LEFT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue FRONT_RIGHT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue REAR_LEFT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue REAR_RIGHT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;

      public static final InvertedValue FRONT_LEFT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue FRONT_RIGHT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue REAR_LEFT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue REAR_RIGHT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;

      public static final InvertedValue FRONT_LEFT_DRIVE_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue FRONT_RIGHT_DRIVE_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue REAR_LEFT_DRIVE_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue REAR_RIGHT_DRIVE_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
    }

    public static final class DevConstants {
      public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 54;
      public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
      public static final int REAR_LEFT_DRIVE_MOTOR_ID = 26;
      public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 1;

      public static final int FRONT_LEFT_TURN_MOTOR_ID = 7;
      public static final int FRONT_RIGHT_TURN_MOTOR_ID = 2;
      public static final int REAR_LEFT_TURN_MOTOR_ID = 5;
      public static final int REAR_RIGHT_TURN_MOTOR_ID = 6;

      public static final int FRONT_LEFT_CANCODER_ID = 11;
      public static final int FRONT_RIGHT_CANCODER_ID = 13;
      public static final int REAR_LEFT_CANCODER_ID = 10;
      public static final int REAR_RIGHT_CANCODER_ID = 12;

      public static final double FRONT_LEFT_ZERO_ANGLE = 0.134765625;
      public static final double FRONT_RIGHT_ZERO_ANGLE = -0.01220703125;
      public static final double REAR_LEFT_ZERO_ANGLE = 0.02099609375;
      public static final double REAR_RIGHT_ZERO_ANGLE = 0.4111328125;

      public static final SensorDirectionValue FRONT_LEFT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue FRONT_RIGHT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue REAR_LEFT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue REAR_RIGHT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;

      public static final InvertedValue FRONT_LEFT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue FRONT_RIGHT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue REAR_LEFT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue REAR_RIGHT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;

      public static final InvertedValue FRONT_LEFT_DRIVE_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue FRONT_RIGHT_DRIVE_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue REAR_LEFT_DRIVE_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue REAR_RIGHT_DRIVE_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
    }

    public static final class CompConstants {
      public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 54;
      public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
      public static final int REAR_LEFT_DRIVE_MOTOR_ID = 26;
      public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 1;

      public static final int FRONT_LEFT_TURN_MOTOR_ID = 7;
      public static final int FRONT_RIGHT_TURN_MOTOR_ID = 2;
      public static final int REAR_LEFT_TURN_MOTOR_ID = 5;
      public static final int REAR_RIGHT_TURN_MOTOR_ID = 6;

      public static final int FRONT_LEFT_CANCODER_ID = 16;
      public static final int FRONT_RIGHT_CANCODER_ID = 13;
      public static final int REAR_LEFT_CANCODER_ID = 10;
      public static final int REAR_RIGHT_CANCODER_ID = 12;

      public static final double FRONT_LEFT_ZERO_ANGLE = 0.134765625;
      public static final double FRONT_RIGHT_ZERO_ANGLE = -0.01220703125;
      public static final double REAR_LEFT_ZERO_ANGLE = 0.02099609375;
      public static final double REAR_RIGHT_ZERO_ANGLE = 0.4111328125;

      public static final SensorDirectionValue FRONT_LEFT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue FRONT_RIGHT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue REAR_LEFT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;
      public static final SensorDirectionValue REAR_RIGHT_CANCODER_REVERSED =
          SensorDirectionValue.CounterClockwise_Positive;

      public static final InvertedValue FRONT_LEFT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue FRONT_RIGHT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue REAR_LEFT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue REAR_RIGHT_TURN_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;

      public static final InvertedValue FRONT_LEFT_DRIVE_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue FRONT_RIGHT_DRIVE_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue REAR_LEFT_DRIVE_MOTOR_REVERSED =
          InvertedValue.Clockwise_Positive;
      public static final InvertedValue REAR_RIGHT_DRIVE_MOTOR_REVERSED =
          InvertedValue.CounterClockwise_Positive;
    }

    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 20; // 20 //10
    public static final double LOW_ANGULAR_SPEED_RADIANS_PER_SECOND = 5; // 5

    public static final double MAX_SPEED_METERS_PER_SECOND = 4.85; // 4.85
    // Constants.getRobot() == RobotType.DEV_ROBOT ? 4.5 : 6.95; // 4.5

    public static final double REPULSOR_TRANSLATION_P = 2.5;
    public static final double REPULSOR_HEADING_P = 2.5;

    public static final double REPULSOR_MAX_VELOCITY = 1.0;
    public static final double REPULSOR_MAX_ACCELERATION = 2.0;
  }

  public class ModuleConstants {
    public static final double GYRO_MAX_PITCH = 30.0; // degrees
    public static final double GYRO_MAX_ROLL = 30.0; // degrees

    public static final double DRIVE_GEAR_RATIO = 6.48;
    // Constants.getRobot() == RobotType.DEV_ROBOT ? 7.13 : 4.59; // 4.59
    public static final double TURN_GEAR_RATIO = 12.1;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.9);

    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_TO_METERS = WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;
    public static final double DRIVE_TO_METERS_PER_SECOND =
        WHEEL_CIRCUMFERENCE_METERS / DRIVE_GEAR_RATIO;

    public double DRIVE_SUPPLY_LIMIT = 55.0; // 55
    public double DRIVE_STATOR_LIMIT = 60.0; // 60

    public static final double TURN_P = 1000.0;
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 25.0;

    public static final double TURN_S = 0.0;
    public static final double TURN_V = 0.0;
    public static final double TURN_A = 0.0;

    public static final double MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND = 15;
    public static final double MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 12;

    public static final double DRIVE_P = 10.4829;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;

    // These values were characterized using our characterization commands.
    public static final double DRIVE_S = 2.7;
    public static final double DRIVE_V = 0.1;
    public static final double DRIVE_A = 0.0;
  }

  public static final ModuleConfig[] aquilaModuleConfigs =
      new ModuleConfig[] {
        new ModuleConfig(
            RobotConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            RobotConstants.FRONT_LEFT_TURN_MOTOR_ID,
            RobotConstants.FRONT_LEFT_CANCODER_ID,
            RobotConstants.FRONT_LEFT_ZERO_ANGLE,
            RobotConstants.FRONT_LEFT_CANCODER_REVERSED,
            RobotConstants.FRONT_LEFT_TURN_MOTOR_REVERSED,
            RobotConstants.FRONT_LEFT_DRIVE_MOTOR_REVERSED),
        new ModuleConfig(
            RobotConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
            RobotConstants.FRONT_RIGHT_TURN_MOTOR_ID,
            RobotConstants.FRONT_RIGHT_CANCODER_ID,
            RobotConstants.FRONT_RIGHT_ZERO_ANGLE,
            RobotConstants.FRONT_RIGHT_CANCODER_REVERSED,
            RobotConstants.FRONT_RIGHT_TURN_MOTOR_REVERSED,
            RobotConstants.FRONT_RIGHT_DRIVE_MOTOR_REVERSED),
        new ModuleConfig(
            RobotConstants.REAR_LEFT_DRIVE_MOTOR_ID,
            RobotConstants.REAR_LEFT_TURN_MOTOR_ID,
            RobotConstants.REAR_LEFT_CANCODER_ID,
            RobotConstants.REAR_LEFT_ZERO_ANGLE,
            RobotConstants.REAR_LEFT_CANCODER_REVERSED,
            RobotConstants.REAR_LEFT_TURN_MOTOR_REVERSED,
            RobotConstants.REAR_LEFT_DRIVE_MOTOR_REVERSED),
        new ModuleConfig(
            RobotConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
            RobotConstants.REAR_RIGHT_TURN_MOTOR_ID,
            RobotConstants.REAR_RIGHT_CANCODER_ID,
            RobotConstants.REAR_RIGHT_ZERO_ANGLE,
            RobotConstants.REAR_RIGHT_CANCODER_REVERSED,
            RobotConstants.REAR_RIGHT_TURN_MOTOR_REVERSED,
            RobotConstants.REAR_RIGHT_DRIVE_MOTOR_REVERSED)
      };

  public static final ModuleConfig[] devModuleConfigs =
      new ModuleConfig[] {
        new ModuleConfig(
            DevConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            DevConstants.FRONT_LEFT_TURN_MOTOR_ID,
            DevConstants.FRONT_LEFT_CANCODER_ID,
            DevConstants.FRONT_LEFT_ZERO_ANGLE,
            DevConstants.FRONT_LEFT_CANCODER_REVERSED,
            DevConstants.FRONT_LEFT_TURN_MOTOR_REVERSED,
            DevConstants.FRONT_LEFT_DRIVE_MOTOR_REVERSED),
        new ModuleConfig(
            DevConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
            DevConstants.FRONT_RIGHT_TURN_MOTOR_ID,
            DevConstants.FRONT_RIGHT_CANCODER_ID,
            DevConstants.FRONT_RIGHT_ZERO_ANGLE,
            DevConstants.FRONT_RIGHT_CANCODER_REVERSED,
            DevConstants.FRONT_RIGHT_TURN_MOTOR_REVERSED,
            DevConstants.FRONT_RIGHT_DRIVE_MOTOR_REVERSED),
        new ModuleConfig(
            DevConstants.REAR_LEFT_DRIVE_MOTOR_ID,
            DevConstants.REAR_LEFT_TURN_MOTOR_ID,
            DevConstants.REAR_LEFT_CANCODER_ID,
            DevConstants.REAR_LEFT_ZERO_ANGLE,
            DevConstants.REAR_LEFT_CANCODER_REVERSED,
            DevConstants.REAR_LEFT_TURN_MOTOR_REVERSED,
            DevConstants.REAR_LEFT_DRIVE_MOTOR_REVERSED),
        new ModuleConfig(
            DevConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
            DevConstants.REAR_RIGHT_TURN_MOTOR_ID,
            DevConstants.REAR_RIGHT_CANCODER_ID,
            DevConstants.REAR_RIGHT_ZERO_ANGLE,
            DevConstants.REAR_RIGHT_CANCODER_REVERSED,
            DevConstants.REAR_RIGHT_TURN_MOTOR_REVERSED,
            DevConstants.REAR_RIGHT_DRIVE_MOTOR_REVERSED)
      };

  public static final ModuleConfig[] compModuleConfigs =
      new ModuleConfig[] {
        new ModuleConfig(
            CompConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            CompConstants.FRONT_LEFT_TURN_MOTOR_ID,
            CompConstants.FRONT_LEFT_CANCODER_ID,
            CompConstants.FRONT_LEFT_ZERO_ANGLE,
            CompConstants.FRONT_LEFT_CANCODER_REVERSED,
            CompConstants.FRONT_LEFT_TURN_MOTOR_REVERSED,
            CompConstants.FRONT_LEFT_DRIVE_MOTOR_REVERSED),
        new ModuleConfig(
            CompConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
            CompConstants.FRONT_RIGHT_TURN_MOTOR_ID,
            CompConstants.FRONT_RIGHT_CANCODER_ID,
            CompConstants.FRONT_RIGHT_ZERO_ANGLE,
            CompConstants.FRONT_RIGHT_CANCODER_REVERSED,
            CompConstants.FRONT_RIGHT_TURN_MOTOR_REVERSED,
            CompConstants.FRONT_RIGHT_DRIVE_MOTOR_REVERSED),
        new ModuleConfig(
            CompConstants.REAR_LEFT_DRIVE_MOTOR_ID,
            CompConstants.REAR_LEFT_TURN_MOTOR_ID,
            CompConstants.REAR_LEFT_CANCODER_ID,
            CompConstants.REAR_LEFT_ZERO_ANGLE,
            CompConstants.REAR_LEFT_CANCODER_REVERSED,
            CompConstants.REAR_LEFT_TURN_MOTOR_REVERSED,
            CompConstants.REAR_LEFT_DRIVE_MOTOR_REVERSED),
        new ModuleConfig(
            CompConstants.REAR_RIGHT_DRIVE_MOTOR_ID,
            CompConstants.REAR_RIGHT_TURN_MOTOR_ID,
            CompConstants.REAR_RIGHT_CANCODER_ID,
            CompConstants.REAR_RIGHT_ZERO_ANGLE,
            CompConstants.REAR_RIGHT_CANCODER_REVERSED,
            CompConstants.REAR_RIGHT_TURN_MOTOR_REVERSED,
            CompConstants.REAR_RIGHT_DRIVE_MOTOR_REVERSED)
      };

  public record ModuleConfig(
      int driveMotorChannel,
      int turnMotorChannel,
      int turnEncoderChannel,
      double angleZero,
      SensorDirectionValue encoderReversed,
      InvertedValue turnReversed,
      InvertedValue driveReversed) {}
}
