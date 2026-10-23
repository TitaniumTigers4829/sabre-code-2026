package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;

public final class VisionConstants {
  public enum Limelight {
    FRONT(FRONT_LEFT_LIMELIGHT_NUMBER, FRONT_LIMELIGHT_NAME, LL4_FOV_MARGIN_OF_ERROR, true);
    // SIDE(SIDE_LIMELIGHT_NUMBER, SIDE_LIMELIGHT_NAME, LL4_FOV_MARGIN_OF_ERROR, true);

    private final int id;
    private final String name;
    private final double accurateFOV;
    private final boolean hasInternalIMU;

    Limelight(int id, String name, double accurateFOV, boolean hasInternalIMU) {
      this.id = id;
      this.name = name;
      this.accurateFOV = accurateFOV;
      this.hasInternalIMU = hasInternalIMU;
    }

    public int getId() {
      return id;
    }

    public String getName() {
      return name;
    }

    public double getAccurateFOV() {
      return accurateFOV;
    }

    public boolean hasInternalIMU() {
      return hasInternalIMU;
    }

    public static Limelight fromId(int id) {
      return switch (id) {
        case 0 -> FRONT;
          // case 1 -> SIDE;
        default -> throw new IllegalArgumentException("Invalid Limelight ID: " + id);
      };
    }
  }

  public static final Transform3d BACK_TRANSFORM =
      new Transform3d(new Translation3d(0.0, 0.0, 0.1865472012), new Rotation3d(0.0, 35, 180.0));
  // x->0.3119324724
  public static final PhotonCamera FRONT_CAMERA = new PhotonCamera(Limelight.FRONT.getName());
  // public static final PhotonCamera ELEVATOR_CAMERA =
  //     new PhotonCamera(Limelight.FRONT_RIGHT.getName());

  public static final int THREAD_SLEEP_MS = 20;

  public static final int POSE_MOVING_AVERAGE_WINDOW_SIZE = 50;

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public static final double VISION_X_POS_TRUST = 0.5; // meters
  public static final double VISION_Y_POS_TRUST = 0.5; // meters
  public static final double VISION_ANGLE_TRUST = Units.degreesToRadians(50); // radians

  // public static final double LL3_FOV_MARGIN_OF_ERROR = 26;
  // public static final double LL3G_FOV_MARGIN_OF_ERROR = 36;

  public static final double LL4_FOV_MARGIN_OF_ERROR = 34.5;
  // only have ll4s for this season

  public static final double MAX_TRANSLATION_DELTA_METERS = 0.8;
  public static final double MAX_ROTATION_DELTA_DEGREES = 50.0;
  public static final double MAX_AMBIGUITY_THRESHOLD = .7;

  public static final double MEGA_TAG_2_DISTANCE_THRESHOLD = 1.5;
  public static final double MEGA_TAG_2_MAX_HEADING_RATE = 100; // degrees/s

  public static final double MEGA_TAG_TRANSLATION_DISCREPANCY_THRESHOLD = .5; // TODO: tune
  public static final double MEGA_TAG_ROTATION_DISCREPANCY_THREASHOLD = 45;

  public static final String FRONT_LIMELIGHT_NAME = "limelight-front";
  public static final int FRONT_LEFT_LIMELIGHT_NUMBER = 0;
  public static final String SIDE_LIMELIGHT_NAME = "limelight-side";
  public static final int SIDE_LIMELIGHT_NUMBER = 1;

  // TODO: these need to be changed, maybe to 10 and 0?
  public static final int DISABLED_THROTTLE = 175;
  public static final int ENABLED_THROTTLE = 5;

  // Constants for port forwarding
  public static final int BASE_PORT = 5800;
  public static final int PORT_RANGE = 10;
  public static final int PORT_OFFSET = 10;
  public static final String LIMELIGHT_DOMAIN = ".local";

  public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
    // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
    {0, 0.02, 0.02, Units.degreesToRadians(180000)}, // 2
    {1.5, 0.1, 0.1, Units.degreesToRadians(180000)}, // 5
    {3, 4.0, 4.0, Units.degreesToRadians(180000)}, // 25
    {4.5, 8.0, 6.0, Units.degreesToRadians(180000)}, // 90
    {8, 10.5, 10.5, Units.degreesToRadians(180000)} // 180
  };

  public static final double[][] TWO_APRIL_TAG_LOOKUP_TABLE = {
    // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
    {0, 0.01, 0.01, Units.degreesToRadians(180000)}, // 0.5
    {1.5, 0.01, 0.01, Units.degreesToRadians(180000)}, // 0.7
    {3, 0.03, 0.06, Units.degreesToRadians(180000)}, // 4
    {4.5, 0.6, 0.12, Units.degreesToRadians(180000)}, // 30
    {8, 0.5, 0.5, Units.degreesToRadians(180000)}, // 90
    {10, 10.0, 10.0, Units.degreesToRadians(180000)} // 90
  };
}
