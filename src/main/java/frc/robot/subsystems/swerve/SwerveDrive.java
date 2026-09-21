package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.extras.logging.Tracer;
import frc.robot.extras.swerve.RepulsorFieldPlanner;
import frc.robot.extras.swerve.setpointGen.SwerveSetpoint;
import frc.robot.extras.swerve.setpointGen.SwerveSetpointGenerator;
import frc.robot.extras.util.AllianceFlipper;
import frc.robot.extras.util.TimeUtil;
import frc.robot.sim.configs.SimSwerveModuleConfig.WheelCof;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroInterface;
import frc.robot.subsystems.swerve.module.ModuleInterface;
import frc.robot.subsystems.vision.VisionConstants;
import java.math.BigDecimal;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

  private final GyroInterface gyroIO;
  private final GyroInputsAutoLogged gyroInputs;
  private final SwerveModule[] swerveModules;

  private final PIDController xChoreoController =
      new PIDController(
          TrajectoryConstants.AUTO_TRANSLATION_P,
          0,
          TrajectoryConstants.AUTO_TRANSLATION_D);

  private final PIDController yChoreoController =
      new PIDController(
          TrajectoryConstants.AUTO_TRANSLATION_P,
          0,
          TrajectoryConstants.AUTO_TRANSLATION_D);

  private final PIDController rotationChoreoController =
      new PIDController(
          TrajectoryConstants.AUTO_THETA_P,
          0,
          TrajectoryConstants.AUTO_THETA_D);

  private Rotation2d rawGyroRotation;

  private final SwerveModulePosition[] lastModulePositions;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final RepulsorFieldPlanner repulsorFieldPlanner =
      new RepulsorFieldPlanner();

  private final ProfiledPIDController xRepulsorController =
      new ProfiledPIDController(
          DriveConstants.REPULSOR_TRANSLATION_P,
          BigDecimal.ZERO.doubleValue(),
          BigDecimal.ZERO.doubleValue(),
          new Constraints(
              DriveConstants.REPULSOR_MAX_VELOCITY,
              DriveConstants.REPULSOR_MAX_ACCELERATION));

  private final ProfiledPIDController yRepulsorController =
      new ProfiledPIDController(
          DriveConstants.REPULSOR_TRANSLATION_P,
          BigDecimal.ZERO.doubleValue(),
          BigDecimal.ZERO.doubleValue(),
          new Constraints(
              DriveConstants.REPULSOR_MAX_VELOCITY,
              DriveConstants.REPULSOR_MAX_ACCELERATION));

  /**
   * Controls the robot's chassis heading.
   *
   * <p>This controller is used for things such as:
   *
   * <ul>
   *   <li>Pointing the robot at the source
   *   <li>Pointing the robot at the hub
   *   <li>Facing a fixed field heading
   *   <li>Following a heading while the driver controls translation
   * </ul>
   */
  private final ProfiledPIDController headingController =
      new ProfiledPIDController(
          DriveConstants.REPULSOR_HEADING_P,
          0.0,
          0.0,
          new Constraints(
              DriveConstants.REPULSOR_MAX_VELOCITY,
              DriveConstants.REPULSOR_MAX_ACCELERATION));

  private final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(
          DriveConstants.MODULE_TRANSLATIONS,
          DCMotor.getFalcon500(1).withReduction(ModuleConstants.DRIVE_GEAR_RATIO),
          DCMotor.getFalcon500(1).withReduction(ModuleConstants.TURN_GEAR_RATIO),
          60,
          58,
          7,
          ModuleConstants.WHEEL_DIAMETER_METERS,
          WheelCof.BLACK_NITRILE.cof,
          0.0);

  private SwerveSetpoint setpoint = SwerveSetpoint.zeroed();

  private Optional<DriverStation.Alliance> alliance;

  private final Alert gyroDisconnectedAlert =
      new Alert(
          "Gyro Hardware Fault",
          Alert.AlertType.kError);

  public SwerveDrive(
      GyroInterface gyroIO,
      ModuleInterface frontLeftModuleIO,
      ModuleInterface frontRightModuleIO,
      ModuleInterface backLeftModuleIO,
      ModuleInterface backRightModuleIO) {

    this.gyroIO = gyroIO;
    this.gyroInputs = new GyroInputsAutoLogged();
    this.rawGyroRotation = new Rotation2d();

    swerveModules =
        new SwerveModule[] {
          new SwerveModule(frontLeftModuleIO, "FrontLeft"),
          new SwerveModule(frontRightModuleIO, "FrontRight"),
          new SwerveModule(backLeftModuleIO, "BackLeft"),
          new SwerveModule(backRightModuleIO, "BackRight")
        };

    lastModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            rawGyroRotation,
            lastModulePositions,
            new Pose2d(),
            VecBuilder.fill(
                DriveConstants.X_POS_TRUST,
                DriveConstants.Y_POS_TRUST,
                DriveConstants.ANGLE_TRUST),
            VecBuilder.fill(
                VisionConstants.VISION_X_POS_TRUST,
                VisionConstants.VISION_Y_POS_TRUST,
                VisionConstants.VISION_ANGLE_TRUST));

    rotationChoreoController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    gyroDisconnectedAlert.set(false);
  }

  @Override
  public void periodic() {
    final double t0 = TimeUtil.getRealTimeSeconds();

    updateSwerveInputs();

    Logger.recordOutput(
        "SystemPerformance/OdometryFetchingTimeMS",
        (TimeUtil.getRealTimeSeconds() - t0) * 1000);

    modulesPeriodic();
  }

  /**
   * Drives the robot using the provided speeds.
   *
   * @param xSpeed X speed in meters per second
   * @param ySpeed Y speed in meters per second
   * @param rotationSpeed Angular velocity in radians per second
   * @param fieldRelative Whether X/Y are field-relative
   */
  public void drive(
      double xSpeed,
      double ySpeed,
      double rotationSpeed,
      boolean fieldRelative) {

    ChassisSpeeds desiredSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rotationSpeed,
                getOdometryAllianceRelativeRotation2d())
            : new ChassisSpeeds(
                xSpeed,
                ySpeed,
                rotationSpeed);

    setpoint =
        setpointGenerator.generateSimpleSetpoint(
            setpoint,
            desiredSpeeds,
            HardwareConstants.LOOP_TIME_SECONDS);

    setModuleStates(setpoint.moduleStates());

    Logger.recordOutput(
        "SwerveStates/DesiredStates",
        setpoint.moduleStates());
  }

  /**
   * Drives the robot using chassis speeds.
   *
   * @param speeds desired chassis speeds
   * @param fieldRelative whether the speeds are field-relative
   */
  public void drive(
      ChassisSpeeds speeds,
      boolean fieldRelative) {

    drive(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        fieldRelative);
  }

  @AutoLogOutput(key = "SwerveState/Speeds")
  public ChassisSpeeds getChassisSpeeds() {
    return setpoint.chassisSpeeds();
  }

  /**
   * Automatically rotates the robot's chassis to a desired field-relative heading.
   *
   * <p>Translation can still be controlled independently by supplying X/Y speeds.
   *
   * <p>For example:
   *
   * <pre>
   * autoAlignHeading(Rotation2d.fromDegrees(90));
   * </pre>
   *
   * will rotate the robot so that it faces 90 degrees on the field.
   *
   * @param targetHeading desired field-relative robot heading
   */
  public void autoAlignHeading(Rotation2d targetHeading) {

    double currentHeading =
        getOdometryRotation2d().getRadians();

    double targetHeadingRadians =
        targetHeading.getRadians();

    double omega =
        headingController.calculate(
            currentHeading,
            targetHeadingRadians);

    Logger.recordOutput(
        "HeadingAlign/CurrentHeading",
        currentHeading);

    Logger.recordOutput(
        "HeadingAlign/TargetHeading",
        targetHeadingRadians);

    Logger.recordOutput(
        "HeadingAlign/Omega",
        omega);

    drive(
        0.0,
        0.0,
        omega,
        true);
  }

  /**
   * Automatically rotates the robot toward a target while allowing
   * field-relative translation.
   *
   * @param translationSupplier supplies field-relative X/Y translation
   * @param targetHeading desired field-relative robot heading
   */
  public void autoAlignHeading(
      Supplier<Translation2d> translationSupplier,
      Rotation2d targetHeading) {

    Translation2d translation =
        translationSupplier.get();

    double currentHeading =
        getOdometryRotation2d().getRadians();

    double targetHeadingRadians =
        targetHeading.getRadians();

    double omega =
        headingController.calculate(
            currentHeading,
            targetHeadingRadians);

    Logger.recordOutput(
        "HeadingAlign/CurrentHeading",
        currentHeading);

    Logger.recordOutput(
        "HeadingAlign/TargetHeading",
        targetHeadingRadians);

    Logger.recordOutput(
        "HeadingAlign/Omega",
        omega);

    drive(
        translation.getX(),
        translation.getY(),
        omega,
        true);
  }

  /**
   * Rotates the robot so that its chassis points directly at a field position.
   *
   * <p>This is useful for things such as aiming at the hub, source, or another
   * field element.
   *
   * @param targetPosition field-relative position to point the robot toward
   */
  public void autoAlignToPoint(
      Translation2d targetPosition) {

    Translation2d robotPosition =
        getEstimatedPose().getTranslation();

    Translation2d robotToTarget =
        targetPosition.minus(robotPosition);

    if (robotToTarget.getNorm() < 0.001) {
      drive(0.0, 0.0, 0.0, true);
      return;
    }

    Rotation2d targetHeading =
        robotToTarget.getAngle();

    autoAlignHeading(targetHeading);
  }

  /**
   * Rotates toward a field position while allowing the driver
   * to control field-relative translation.
   *
   * @param translationSupplier driver translation supplier
   * @param targetPosition field-relative position to face
   */
  public void autoAlignToPoint(
      Supplier<Translation2d> translationSupplier,
      Translation2d targetPosition) {

    Translation2d robotPosition =
        getEstimatedPose().getTranslation();

    Translation2d robotToTarget =
        targetPosition.minus(robotPosition);

    if (robotToTarget.getNorm() < 0.001) {
      Translation2d translation =
          translationSupplier.get();

      drive(
          translation.getX(),
          translation.getY(),
          0.0,
          true);

      return;
    }

    Rotation2d targetHeading =
        robotToTarget.getAngle();

    autoAlignHeading(
        translationSupplier,
        targetHeading);
  }

  /**
   * Allows PID control of the chassis rotation.
   *
   * @param targetHeading desired field-relative heading
   */
  public void setHeading(Rotation2d targetHeading) {
    autoAlignHeading(targetHeading);
  }

  /**
   * Runs characterization on voltage.
   *
   * @param volts voltage to apply
   */
  public void runCharacterizationVoltage(double volts) {
    for (SwerveModule module : swerveModules) {
      module.setVoltage(Volts.of(-volts));
    }
  }

  /**
   * Runs characterization on current.
   *
   * @param amps current to apply
   */
  public void runCharacterizationCurrent(double amps) {
    for (SwerveModule module : swerveModules) {
      module.setCurrent(Amps.of(-amps));
    }
  }

  /**
   * Drives the robot in place for wheel radius characterization.
   *
   * @param omegaSpeed angular speed
   */
  public void runWheelRadiusCharacterization(double omegaSpeed) {
    drive(
        0,
        0,
        omegaSpeed,
        false);
  }

  /**
   * Gets the wheel radius characterization positions.
   *
   * @return wheel positions in radians
   */
  public double[] getWheelRadiusCharacterizationPosition() {

    double[] wheelPositions =
        new double[swerveModules.length];

    for (int i = 0; i < 4; i++) {
      wheelPositions[i] =
          swerveModules[i].getDrivePositionRadians();
    }

    return wheelPositions;
  }

  /**
   * Gets the total characterization velocity.
   *
   * @return summed module velocity
   */
  public double getCharacterizationVelocity() {

    double velocity = 0.0;

    for (SwerveModule module : swerveModules) {
      velocity += module.getCharacterizationVelocity();
    }

    return velocity;
  }

  /**
   * Follows a Choreo SwerveSample.
   *
   * @param sample trajectory sample
   */
  public void followSwerveSample(
      SwerveSample sample) {

    xChoreoController.reset();
    yChoreoController.reset();
    rotationChoreoController.reset();

    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            sample.vx
                + xChoreoController.calculate(
                    getEstimatedPose().getX(),
                    sample.x),

            sample.vy
                + yChoreoController.calculate(
                    getEstimatedPose().getY(),
                    sample.y),

            sample.omega
                + rotationChoreoController.calculate(
                    getOdometryRotation2d().getRadians(),
                    sample.heading),

            getOdometryRotation2d());

    Logger.recordOutput(
        "Trajectories/CurrentX",
        getEstimatedPose().getX());

    Logger.recordOutput(
        "Trajectories/DesiredX",
        sample.x);

    Logger.recordOutput(
        "Trajectories/vx",
        sample.vx);

    Logger.recordOutput(
        "Trajectories/omega",
        sample.omega);

    Logger.recordOutput(
        "Trajectories/headingOutput",
        rotationChoreoController.calculate(
            getOdometryRotation2d().getRadians(),
            sample.heading));

    Logger.recordOutput(
        "Trajectories/desiredHeading",
        sample.heading);

    drive(
        chassisSpeeds.unaryMinus(),
        false);
  }

  /** Runs all SwerveModule periodic methods. */
  private void modulesPeriodic() {
    for (SwerveModule module : swerveModules) {
      module.periodic();
    }
  }

  /**
   * Returns whether the supplied speeds are zero.
   *
   * @param speeds chassis speeds
   * @return whether all chassis speeds are zero
   */
  public boolean getZeroedSpeeds(
      ChassisSpeeds speeds) {

    return speeds.vxMetersPerSecond == 0
        && speeds.vyMetersPerSecond == 0
        && speeds.omegaRadiansPerSecond == 0;
  }

  /**
   * Returns the gyro heading in degrees.
   *
   * @return heading in degrees
   */
  public double getHeading() {
    return gyroInputs.yawDegrees;
  }

  /**
   * Gets the gyro angular velocity.
   *
   * @return angular velocity in degrees/sec
   */
  public double getGyroRate() {
    return gyroInputs.yawVelocityDegreesPerSecond;
  }

  /**
   * Gets the gyro roll.
   *
   * @return roll in degrees
   */
  public double getGyroRoll() {
    return gyroInputs.rollDegrees;
  }

  /**
   * Gets the gyro pitch.
   *
   * @return pitch in degrees
   */
  public double getGyroPitch() {
    return gyroInputs.pitchDegrees;
  }

  /**
   * Gets the gyro heading as Rotation2d.
   *
   * @return gyro rotation
   */
  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(
        getHeading());
  }

  /**
   * Gets the gyro heading relative to the driver's alliance.
   *
   * @return alliance-relative gyro heading
   */
  public Rotation2d getGyroFieldRelativeRotation2d() {
    return Rotation2d.fromDegrees(
        getHeading()
            + getAllianceAngleOffset());
  }

  /**
   * Gets the alliance angle offset.
   *
   * @return 0 degrees for blue, 180 degrees for red
   */
  public double getAllianceAngleOffset() {

    alliance =
        DriverStation.getAlliance();

    return alliance.isPresent()
            && alliance.get()
                == DriverStation.Alliance.Red
        ? 180.0
        : 0.0;
  }

  /**
   * Resets gyro heading.
   */
  public void zeroHeading() {
    gyroIO.reset();
  }

  /**
   * Gets the estimated robot pose.
   *
   * @return estimated field-relative pose
   */
  @AutoLogOutput(key = "Odometry/Odometry")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the robot's odometry heading.
   *
   * @return odometry rotation
   */
  public Rotation2d getOdometryRotation2d() {
    return getEstimatedPose().getRotation();
  }

  /**
   * Gets the odometry heading relative to the alliance.
   *
   * @return alliance-relative odometry heading
   */
  public Rotation2d getOdometryAllianceRelativeRotation2d() {
    return getEstimatedPose()
        .getRotation()
        .plus(
            Rotation2d.fromDegrees(
                getAllianceAngleOffset()));
  }

  /**
   * Sets module states.
   *
   * @param desiredStates desired module states
   */
  public void setModuleStates(
      SwerveModuleState[] desiredStates) {

    for (int i = 0; i < 4; i++) {
      swerveModules[i]
          .setOptimizedDesiredState(
              desiredStates[i]);
    }
  }

  /** Sets the modules into an X stance. */
  public void setXStance() {

    Rotation2d[] swerveHeadings =
        new Rotation2d[swerveModules.length];

    for (int i = 0; i < 4; i++) {
      swerveHeadings[i] =
          Rotation2d.fromDegrees(45);
    }

    DriveConstants.DRIVE_KINEMATICS
        .resetHeadings(swerveHeadings);

    for (int i = 0; i < 4; i++) {
      swerveModules[i].stopModule();
    }
  }

  /**
   * Updates pose estimation using swerve measurements.
   */
  public void addPoseEstimatorSwerveMeasurement() {

    final SwerveModulePosition[] modulePositions =
        getModulePositions();

    final SwerveModulePosition[] moduleDeltas =
        getModulesDelta(modulePositions);

    if (gyroInputs.isConnected) {

      rawGyroRotation =
          getGyroRotation2d();

    } else {

      Twist2d twist =
          DriveConstants.DRIVE_KINEMATICS
              .toTwist2d(moduleDeltas);

      rawGyroRotation =
          rawGyroRotation.plus(
              new Rotation2d(twist.dtheta));
    }

    poseEstimator.updateWithTime(
        TimeUtil.getLogTimeSeconds(),
        rawGyroRotation,
        modulePositions);
  }

  /**
   * Gets module position deltas.
   *
   * @param freshModulesPosition latest positions
   * @return module position deltas
   */
  private SwerveModulePosition[] getModulesDelta(
      SwerveModulePosition[] freshModulesPosition) {

    SwerveModulePosition[] deltas =
        new SwerveModulePosition[swerveModules.length];

    for (int moduleIndex = 0;
        moduleIndex < 4;
        moduleIndex++) {

      final double deltaDistanceMeters =
          freshModulesPosition[moduleIndex].distanceMeters
              - lastModulePositions[moduleIndex]
                  .distanceMeters;

      deltas[moduleIndex] =
          new SwerveModulePosition(
              deltaDistanceMeters,
              freshModulesPosition[moduleIndex].angle);

      lastModulePositions[moduleIndex] =
          freshModulesPosition[moduleIndex];
    }

    return deltas;
  }

  /**
   * Gets measured module states.
   *
   * @return module states
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states =
        new SwerveModuleState[
            swerveModules.length];

    for (int i = 0; i < states.length; i++) {
      states[i] =
          swerveModules[i].getMeasuredState();
    }

    return states;
  }

  /**
   * Gets module positions.
   *
   * @return module positions
   */
  private SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions =
        new SwerveModulePosition[
            swerveModules.length];

    for (int i = 0; i < positions.length; i++) {
      positions[i] =
          swerveModules[i].getPosition();
    }

    return positions;
  }

  /**
   * Resets the estimated pose.
   *
   * @param pose new robot pose
   */
  public void resetEstimatedPose(
      Pose2d pose) {

    poseEstimator.resetPosition(
        rawGyroRotation,
        getModulePositions(),
        pose);
  }

  // --------------------------------------------------------------------------
  // HUB / SHOOTING
  // --------------------------------------------------------------------------

  public double getDistanceFromAllianceHub() {

    if (AllianceFlipper.isBlue()) {

      return FieldConstants.BLUE_HUB_CENTER
          .getDistance(
              poseEstimator
                  .getEstimatedPosition()
                  .getTranslation());

    } else {

      return FieldConstants.RED_HUB_CENTER
          .getDistance(
              poseEstimator
                  .getEstimatedPosition()
                  .getTranslation());
    }
  }

  public double getShootingAngle() {

    return Math.atan(
        (FieldConstants.HUB_HEIGHT_METERS
                - ShooterConstants.SHOOTER_HEIGHT_FROM_GROUND)
            / getDistanceFromAllianceHub());
  }

  // --------------------------------------------------------------------------
  // SOURCE ALIGNMENT
  // --------------------------------------------------------------------------

  /**
   * Checks whether the robot is near the source.
   *
   * @return whether the robot is near the source
   */
  @AutoLogOutput
  public boolean nearSource() {

    double maxX = 3.5;

    if (AllianceFlipper.isRed()) {

      return poseEstimator
              .getEstimatedPosition()
              .getX()
          > (FieldConstants.FIELD_WIDTH_METERS
              - maxX);

    } else {

      return poseEstimator
              .getEstimatedPosition()
              .getX()
          < maxX;
    }
  }

  /**
   * Automatically aligns the robot's chassis to face the source.
   *
   * <p>The driver can still control field-relative translation.
   *
   * @param translationalControlSupplier driver translation supplier
   */
  public void sourceAlign(
      Supplier<Translation2d> translationalControlSupplier) {

    double targetAngle =
        Units.degreesToRadians(54);

    if (AllianceFlipper.isRed()) {
      targetAngle =
          Math.PI - targetAngle;
    }

    if (poseEstimator
            .getEstimatedPosition()
            .getY()
        > FieldConstants.FIELD_WIDTH_METERS / 2) {

      targetAngle *= -1;
    }

    autoAlignHeading(
        translationalControlSupplier,
        Rotation2d.fromRadians(targetAngle));
  }

  // --------------------------------------------------------------------------
  // REPULSOR FIELD
  // --------------------------------------------------------------------------

  /**
   * Follows the repulsor field to a goal.
   *
   * @param goal desired pose
   */
  public void followRepulsorField(
      Pose2d goal) {

    followRepulsorField(
        goal,
        null);
  }

  /**
   * Follows the repulsor field to a goal.
   *
   * @param goal desired pose
   * @param nudgeSupplier optional nudge supplier
   */
  public void followRepulsorField(
      Pose2d goal,
      Supplier<Translation2d> nudgeSupplier) {

    repulsorFieldPlanner.setGoal(
        goal.getTranslation());

    xRepulsorController.reset(
        poseEstimator
            .getEstimatedPosition()
            .getX());

    yRepulsorController.reset(
        poseEstimator
            .getEstimatedPosition()
            .getY());

    headingController.reset(
        poseEstimator
            .getEstimatedPosition()
            .getRotation()
            .getRadians());

    Logger.recordOutput(
        "Repulsor/Goal",
        goal);

    ChassisSpeeds feedback =
        new ChassisSpeeds(

            xRepulsorController.calculate(
                poseEstimator
                    .getEstimatedPosition()
                    .getX(),
                goal.getX()),

            yRepulsorController.calculate(
                poseEstimator
                    .getEstimatedPosition()
                    .getY(),
                goal.getY()),

            headingController.calculate(
                poseEstimator
                    .getEstimatedPosition()
                    .getRotation()
                    .getRadians(),
                goal.getRotation()
                    .getRadians()));

    Transform2d error =
        goal.minus(
            poseEstimator
                .getEstimatedPosition());

    Logger.recordOutput(
        "Repulsor/Error",
        error);

    Logger.recordOutput(
        "Repulsor/Feedback",
        feedback);

    ChassisSpeeds outputFieldRelative =
        feedback;

    if (nudgeSupplier != null) {

      Translation2d nudge =
          nudgeSupplier.get();

      if (nudge.getNorm() > 0.1) {

        double nudgeScalar =
            Math.min(
                    error.getTranslation().getNorm()
                        / 3,
                    1)
                * Math.min(
                    error.getTranslation().getNorm()
                        / 3,
                    1)
                * DriveConstants.MAX_SPEED_METERS_PER_SECOND;

        if (AllianceFlipper.isRed()) {
          nudge =
              new Translation2d(
                  -nudge.getX(),
                  -nudge.getY());
        }

        nudgeScalar *=
            Math.abs(
                nudge
                    .getAngle()
                    .minus(
                        new Rotation2d(
                            outputFieldRelative
                                .vxMetersPerSecond,
                            outputFieldRelative
                                .vyMetersPerSecond))
                    .getSin());

        outputFieldRelative.vxMetersPerSecond +=
            nudge.getX() * nudgeScalar;

        outputFieldRelative.vyMetersPerSecond +=
            nudge.getY() * nudgeScalar;
      }
    }

    ChassisSpeeds outputRobotRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            outputFieldRelative,
            poseEstimator
                .getEstimatedPosition()
                .getRotation());

    drive(
        outputRobotRelative.unaryMinus(),
        false);
  }

  // --------------------------------------------------------------------------
  // VISION
  // --------------------------------------------------------------------------

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionMeasurement vision pose
   * @param currentTimeStampSeconds timestamp
   */
  public void addPoseEstimatorVisionMeasurement(
      Pose2d visionMeasurement,
      double currentTimeStampSeconds) {

    poseEstimator.addVisionMeasurement(
        visionMeasurement,
        currentTimeStampSeconds);
  }

  /**
   * Sets vision measurement confidence.
   *
   * @param xStandardDeviation X standard deviation
   * @param yStandardDeviation Y standard deviation
   * @param thetaStandardDeviation heading standard deviation
   */
  public void setPoseEstimatorVisionConfidence(
      double xStandardDeviation,
      double yStandardDeviation,
      double thetaStandardDeviation) {

    poseEstimator.setVisionMeasurementStdDevs(
        VecBuilder.fill(
            xStandardDeviation,
            yStandardDeviation,
            thetaStandardDeviation));
  }

  // --------------------------------------------------------------------------
  // GYRO / INPUTS
  // --------------------------------------------------------------------------

  /** Updates gyro and module inputs. */
  private void updateSwerveInputs() {

    for (SwerveModule module : swerveModules) {
      module.updateOdometryInputs();
    }

    gyroIO.updateInputs(gyroInputs);

    Logger.processInputs(
        "Drive/Gyro",
        gyroInputs);

    Tracer.traceFunc(
        "Gyro",
        () -> gyroIO.updateInputs(gyroInputs));

    gyroDisconnectedAlert.set(
        !gyroInputs.isConnected);
  }
}
