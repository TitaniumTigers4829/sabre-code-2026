package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.math.interpolation.SingleLinearInterpolator;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class DumperShootCommand extends Command {
  private final ShooterSubsystem shooter;
  private final SwerveDrive drive;
  private final BooleanSupplier overridingHood;

  Pose2d robotPose;
  double targetX;
  Pose2d offsettedTarget;
  Translation2d turretPose;
  double deltaX = 0;
  double deltaY = 0;
  double velocityXOffset = 0;
  double velocityYOffset = 0;
  double velocityOmega = 0;
  double omegaXOffset = 0;
  double omegaYOffset = 0;
  double tAir = 1; // calculate this with utility class or interpolating tree
  ChassisSpeeds fieldRelative;
  double iterativeDistance = 0;
  double distance;
  double dampener;
  boolean isAimingProperly = false;

  public SingleLinearInterpolator timeInAirLookupTable =
      new SingleLinearInterpolator(
          new double[][] {
            {1.5, 1.025},
            {2.0, 1.14},
            // {2.5, 1.24},
            {3.0, 1.3},
            {3.5, 1.37},
            {4.0, 1.4},
            {4.5, 1.42},
            {5.0, 1.45},
            {6, 2},
            {10, 3}
          });

  public DumperShootCommand(
      SwerveDrive drive, ShooterSubsystem shooter, BooleanSupplier overridingHood) {
    this.drive = drive;
    this.shooter = shooter;
    this.overridingHood = overridingHood;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Sets hub position based on the alliance
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      targetX = 14;
    } else {
      targetX = 3;
    }
  }

  @Override
  public void execute() {
    dampener = -1;

    turretPose = robotPose.getTranslation();
    robotPose = drive.getEstimatedPose();
    fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), robotPose.getRotation());

    velocityOmega = drive.getChassisSpeeds().omegaRadiansPerSecond;

    double targetY = robotPose.getY() < FieldConstants.FIELD_WIDTH_METERS / 2 ? 1.6 : 6.3;

    Translation2d targetPosition = new Translation2d(targetX, targetY);

    iterativeDistance = turretPose.getDistance(targetPosition);

    for (int i = 0; i < 20; i++) {
      tAir = timeInAirLookupTable.getLookupValue(iterativeDistance);

      velocityXOffset = fieldRelative.vxMetersPerSecond * tAir * dampener;
      velocityYOffset = fieldRelative.vyMetersPerSecond * tAir * dampener;

      offsettedTarget =
          new Pose2d(
              targetPosition.getX() - velocityXOffset - omegaXOffset,
              targetPosition.getY() - velocityYOffset - omegaYOffset,
              new Rotation2d());

      iterativeDistance = offsettedTarget.getTranslation().getDistance(turretPose);
    }

    double targetAngleRad = Math.atan2(deltaY, deltaX);
    // Wrap to [-pi, pi]
    targetAngleRad = Math.atan2(Math.sin(targetAngleRad), Math.cos(targetAngleRad));
    Rotation2d desiredHeading = new Rotation2d(targetAngleRad);

    drive.autoAlignHeading(desiredHeading);

    distance = offsettedTarget.getTranslation().getDistance(turretPose);

    shooter.setPercentOutput(distance, false);

    SmartDashboard.putNumber("DistFromHub", distance);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShoot();
  }
}
