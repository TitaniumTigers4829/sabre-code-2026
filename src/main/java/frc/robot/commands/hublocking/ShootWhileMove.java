package frc.robot.commands.hublocking;

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
import org.littletonrobotics.junction.Logger;

public class ShootWhileMove extends Command {
  private final ShooterSubsystem shooter;
  private final SwerveDrive drive;
  private final BooleanSupplier useOneMotor;

  Pose2d robotPose;
  Translation2d targetPosition;
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
            {5.0, 1.45}
          });

  public ShootWhileMove(SwerveDrive drive, ShooterSubsystem shooter, BooleanSupplier useOneMotor) {
    this.drive = drive;
    this.shooter = shooter;
    this.useOneMotor = useOneMotor;
  }

  // Still lets you make a "normal" one for if you never want to override e.g. autos
  public ShootWhileMove(SwerveDrive drive, ShooterSubsystem shooter) {
    this(drive, shooter, () -> false);
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Sets hub position based on the alliance
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      targetPosition = FieldConstants.RED_HUB_CENTER;
    } else {
      targetPosition = FieldConstants.BLUE_HUB_CENTER;
    }

    shooter.startingShoot();
  }

  @Override
  public void execute() {
    dampener = -0.5;
    robotPose = drive.getEstimatedPose();

    fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), robotPose.getRotation());
    // fieldRelative = new ChassisSpeeds(0, 0, 0);

    velocityOmega = drive.getChassisSpeeds().omegaRadiansPerSecond;
    // velocityOmega = 0;

    iterativeDistance = turretPose.getDistance(targetPosition);

    SmartDashboard.putNumber("hub dist", turretPose.getDistance(targetPosition));

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

    distance = offsettedTarget.getTranslation().getDistance(turretPose);

    deltaX = offsettedTarget.getX() - turretPose.getX();
    deltaY = offsettedTarget.getY() - turretPose.getY();

    double turretAngleRad = Math.atan2(deltaY, deltaX) - robotPose.getRotation().getRadians();
    // Wrap to [-pi, pi]
    turretAngleRad = Math.atan2(Math.sin(turretAngleRad), Math.cos(turretAngleRad));
    double desiredHeading = turretAngleRad / (2.0 * Math.PI);

    desiredHeading -= 0.25; // .25 is because we zero it facing left instead of forward

    SmartDashboard.putBoolean("aiming properly", isAimingProperly);

    if (isAimingProperly) {
      shooter.setPercentOutput(distance, useOneMotor.getAsBoolean());
    } else {
      shooter.stopShoot();
    }
    shooter.setPercentOutput(distance, useOneMotor.getAsBoolean());

    Logger.recordOutput("Shoot on move At Hub/Desired Hub", offsettedTarget);

    Logger.recordOutput("Shoot on move At Hub/Distance to Desire Hub", distance);

    Logger.recordOutput("Shoot on move At Hub/Desired Rotation", desiredHeading);
    Logger.recordOutput("Shoot on move At Hub/ XOffset", deltaX);
    Logger.recordOutput("Shoot on move At Hub/ YOffset", deltaY);
    Logger.recordOutput("Shoot on move At Hub/ OmegaXOffset", omegaXOffset);
    Logger.recordOutput("Shoot on move At Hub/ OmegaYOffset", omegaYOffset);
    Logger.recordOutput("Shoot on move At Hub/ LinearXOffset", velocityXOffset);
    Logger.recordOutput("Shoot on move At Hub/ LinearYOffset", velocityYOffset);
    Logger.recordOutput("Shoot on move At Hub/tAir", tAir);
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
