// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCommand extends Command {
  /** Creates a new AlignCommand. */
  private final SwerveDrive s_Swerve;

  private Pose2d targetPose;
  private final boolean precise;
  private PIDController oPID;
  private final double speed;
  // private final double maxVisionDiff;
  private final boolean flipIfRed;
  // private final SwerveAlign swerveAlign;
  private VisionSubsystem vision;
  private boolean isAligned;
  private SwerveSample swerveSample;

  public AlignCommand(
      SwerveDrive s_Swerve,
      VisionSubsystem visionSubsystem,
      Pose2d targetPose,
      boolean flipIfRed,
      boolean precise,
      double speed) {
    this.s_Swerve = s_Swerve;
    this.targetPose = targetPose;
    this.precise = precise;
    this.speed = speed;
    this.flipIfRed = flipIfRed;
    this.vision = vision;
    // this.swerveAlign = swerveAlign;
    this.isAligned = isAligned;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {

    super.initialize();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Sets hub position based on the alliance
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      targetPose = new Pose2d(FieldConstants.RED_HUB_CENTER, new Rotation2d());
    } else {
      targetPose = new Pose2d(FieldConstants.BLUE_HUB_CENTER, new Rotation2d());
    }

    //     oPID = new PIDController(SwerveConstants.DriveConstants.REPULSOR_HEADING_P, 0, 0);

    //     // oPID.setIZone(PIDSwerveConstants.positionIZone); // Only use Integral term within this
    // range
    //     // yPID.setIntegratorRange(-PIDSwerveConstants.positionKS * 2,
    // PIDSwerveConstants.positionKS * 2);
    //     oPID.setSetpoint(Units.metersToInches(targetPose.getRotation().getRadians()));
    //     if (precise) {
    //         oPID.setTolerance(5, 5.0); // Inches per second
    //     } else {
    //         oPID.setTolerance(10);
    //     }

    // isAligned =
    //     Math.abs(oPID.getError()) <= oPID.getErrorTolerance() &&
    //         (precise ? SwerveAlign.aligned() : SwerveAlign.aligned(3));

    // // oPID.reset();

    // SwerveAlign.errorReset();
    // SwerveAlign.setTarget(targetPose.getRotation());
    // // Use addRequirements() here to declare subsystem dependencies.
    // Logger.recordOutput("PIDSwerve/Pose Target", targetPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = s_Swerve.getEstimatedPose();
    Translation2d position = pose.getTranslation();
    Rotation2d rotation = targetPose.getRotation();

    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(s_Swerve.getChassisSpeeds(), pose.getRotation());
    // fieldRelative = new ChassisSpeeds(0, 0, 0);

    double velocityOmega = s_Swerve.getChassisSpeeds().omegaRadiansPerSecond;
    // velocityOmega = 0;

    double velocityXOffset = fieldRelative.vxMetersPerSecond;
    double velocityYOffset = fieldRelative.vyMetersPerSecond;

    Pose2d offsettedTarget =
        new Pose2d(
            targetPose.getX() - velocityXOffset,
            targetPose.getY() - velocityYOffset,
            new Rotation2d());

    double deltaX = offsettedTarget.getX() - pose.getX();
    double deltaY = offsettedTarget.getY() - pose.getY();

    double targetAngleRad = Math.atan2(deltaY, deltaX);
    // Wrap to [-pi, pi]
    targetAngleRad = Math.atan2(Math.sin(targetAngleRad), Math.cos(targetAngleRad));
    Rotation2d desiredHeading = new Rotation2d(targetAngleRad);

    swerveSample =
        new SwerveSample(
            0,
            pose.getX(),
            pose.getY(),
            desiredHeading.getRadians(),
            velocityXOffset,
            velocityYOffset,
            0,
            0,
            0,
            0,
            null,
            null);

    FollowSwerveSampleCommand swerveSampleCo =
        new FollowSwerveSampleCommand(s_Swerve, vision, swerveSample);
    swerveSampleCo.execute();
    /* Drive */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
