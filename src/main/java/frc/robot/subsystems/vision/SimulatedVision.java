package frc.robot.subsystems.vision;

// import com.titaniumtigers4829.utils.NTUtils;
import edu.wpi.first.math.geometry.*;
// import edu.wpi.first.networktables.NetworkTable;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
// import java.util.ArrayList;
// import java.util.Arrays;
// import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

/**
 *
 *
 * <h3>Simulates the vision system.</h3>
 *
 * <p>SimulatedVision is a class that simulates the vision system and extends {@link
 * PhysicalVision}. It uses the PhotonVision library to send simulated NetworkTables data to the
 * limelight tables.
 *
 * <p><b>See</b>: <a
 *
 * <p>href="https://github.com/PhotonVision/photonvision/blob/2a6fa1b6ac81f239c59d724da5339f608897c510/photonlib-java-examples/swervedriveposeestsim/src/main/java/frc/robot/Vision.java">PhotonVision
 * example</a> for an example of odometry simulation using PhotonVision.
 *
 * @author @Ishan1522
 */
public class SimulatedVision extends PhysicalVision {
  PhotonCameraSim shooterCameraSim;

  private final int kResWidth = 1280;
  private final int kResHeight = 800;

  public SimulatedVision(Supplier<VisionSystemSim> pVisionSim) {
    super();
    // this.robotSimulationPose = robotSimulationPose;
    // Create the vision system simulation which handles cameras and targets on the
    // field.
    // visionSim = new VisionSystemSim("main");

    // Add all the AprilTags inside the tag layout as visible targets to this
    // simulated field.

    // Create simulated camera properties. These can be set to mimic your actual
    // camera.
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(kResWidth, kResHeight, Rotation2d.fromDegrees(97.7));
    cameraProperties.setCalibError(0.35, 0.10);
    cameraProperties.setFPS(15);
    cameraProperties.setAvgLatencyMs(20);
    cameraProperties.setLatencyStdDevMs(5);

    // Create a PhotonCameraSim which will update the linked PhotonCamera's values
    // with visible
    // targets.
    // Instance variables
    // shooterCameraSim = new PhotonCameraSim(getSimulationCamera(Limelight.SIDE), cameraProperties);
    // pVisionSim
    //     .get()
    //     .addCamera(shooterCameraSim, VisionConstants.BACK_TRANSFORM); // check inverse things

    // Enable the raw and processed streams. (http://localhost:1181 / 1182)
    shooterCameraSim.enableRawStream(true);
    shooterCameraSim.enableProcessedStream(true);

    // // Enable drawing a wireframe visualization of the field to the camera streams.
    // // This is extremely resource-intensive and is disabled by default.
    shooterCameraSim.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    // Abuse the updateInputs periodic call to update the sim

    // for (Limelight limelight : Limelight.values()) {
    //   writeToTable(
    //       getSimulationCamera(Limelight.FRONT).getAllUnreadResults(),
    //       getLimelightTable(Limelight.FRONT),
    //       Limelight.SIDE);
    // }
    // super.updateInputs(inputs);
  }

  /**
   * Writes photonvision results to the limelight network table
   *
   * @param results The simulated photonvision pose estimation results
   * @param table The Limelight table to write to
   * @param limelight The Limelight to write for
   */
  /*
  private void writeToTable(
      List<PhotonPipelineResult> results, NetworkTable table, Limelight limelight) {
    // write to ll table
    for (PhotonPipelineResult result : results) {
      if (result.getMultiTagResult().isPresent()) {
        Transform3d best = result.getMultiTagResult().get().estimatedPose.best;
        List<Double> pose_data =
            new ArrayList<>(
                Arrays.asList(
                    best.getX(), // 0: X
                    best.getY(), // 1: Y
                    best.getZ(), // 2: Z,
                    best.getRotation().getX(), // 3: roll
                    best.getRotation().getY(), // 4: pitch
                    best.getRotation().getZ(), // 5: yaw
                    result.metadata.getLatencyMillis(), // 6: latency ms,
                    (double)
                        result.getMultiTagResult().get().fiducialIDsUsed.size(), // 7: tag count
                    0.0, // 8: tag span
                    0.0, // 9: tag dist
                    result.getBestTarget().getArea() // 10: tag area
                    ));
        // Add RawFiducials
        // This is super inefficient but it's sim only, who cares.
        for (PhotonTrackedTarget target : result.targets) {
          pose_data.add((double) target.getFiducialId()); // 0: id
          pose_data.add(target.getYaw()); // 1: txnc
          pose_data.add(target.getPitch()); // 2: tync
          pose_data.add(target.getArea()); // 3: ta
          pose_data.add(0.0); // 4: distToCamera
          pose_data.add(0.0); // 5: distToRobot
          pose_data.add(target.getPoseAmbiguity()); // 6: ambiguity
        }

        table
            .getEntry("botpose_wpiblue")
            .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
        table
            .getEntry("botpose_orb_wpiblue")
            .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());

        table.getEntry("tv").setInteger(result.hasTargets() ? 1 : 0);
        table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
      }
    }
  }
  */

  /**
   * Gets the PhotonCamera for the given Limelight
   *
   * @param limelight The Limelight to get the camera for
   * @return A PhotonCamera object for the given Limelight
   */
  // private PhotonCamera getSimulationCamera(Limelight limelight) {
  //   return switch (limelight) {
  //     case SIDE -> VisionConstants.FRONT_CAMERA;
  //     default -> throw new IllegalArgumentException("Invalid limelight camera " + limelight);
  //   };
  // }

  /**
   * Gets the Limelight network table
   *
   * @param limelight The Limelight to get the table for
   * @return The network table of the Limelight
   */
  /*
  private NetworkTable getLimelightTable(Limelight limelight) {
    return switch (limelight) {
      case SIDE -> NTUtils.getLimelightNetworkTable(Limelight.SIDE.getName());
        // case SIDE -> NTUtils.getLimelightNetworkTable(Limelight.SIDE.getName());
      default -> throw new IllegalArgumentException("Invalid limelight " + limelight);
    };
  }
  */

  @Override
  public void setOdometryInfo(double headingDegrees, double headingRateDegrees) {
    super.setOdometryInfo(headingDegrees, headingRateDegrees);
  }
}
