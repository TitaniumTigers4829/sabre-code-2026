package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.HardwareConstants;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.hublocking.ShootWhileMove;
import frc.robot.commands.intake.DefenseCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakePivotBounceLower;
import frc.robot.commands.intake.IntakePivotDownCommand;
import frc.robot.commands.intake.IntakePivotUpCommand;
import frc.robot.commands.intake.MoveIntakeUpCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.intake.ReverseKickerAndRollers;
// import frc.robot.commands.intake.ReverseSpindexerCommand;
import frc.robot.commands.shooter.DumperShootCommand;
import frc.robot.extras.util.JoystickUtil;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.PhysicalIntake;
import frc.robot.subsystems.shooter.PhysicalShooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.gyro.PhysicalGyroPigeon;
import frc.robot.subsystems.swerve.module.PhysicalModule;
import frc.robot.subsystems.vision.PhysicalVision;
import frc.robot.subsystems.vision.VisionInterface;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private SwerveDrive swerveDrive;
  private VisionSubsystem visionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;

  private Autos autos;
  private Command autoCommand;

  public Robot() {
    checkGit();
    setupLogging();
    setupSubsystems();
    setupAuto();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority and real time to improve loop timing
    Threads.setCurrentThreadPriority(true, HardwareConstants.HIGH_THREAD_PRIORITY);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Updates autos while the robot is enabled
    // autos.update();

    // Return to normal thread priority without real time
    Threads.setCurrentThreadPriority(false, HardwareConstants.LOW_THREAD_PRIORITY);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    autos.update();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoCommand = autos.getSelectedCommand();
    autoCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    autoCommand.cancel();
    autos.clear();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureDriverController();
    configureOperatorController();
  }

  /**
   * Callback for the auto align command, will rumble the controllers when aligned
   *
   * @param isAligned whether the robot is aligned
   */
  /*
  private void alignCallback(boolean isAligned) {
    if (isAligned) {
      driverController.setRumble(RumbleType.kBothRumble, 0.5);
      operatorController.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      driverController.setRumble(RumbleType.kBothRumble, 0.0);
      operatorController.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }
  */

  /** Configures the driver controller buttons and axes to control the robot */
  private void configureDriverController() {
    // Driver Left Stick
    DoubleSupplier driverLeftStick[] =
        new DoubleSupplier[] {
          () ->
              JoystickUtil.modifyAxisPolar(
                  driverController::getLeftX, driverController::getLeftY, 3)[1],
          () ->
              JoystickUtil.modifyAxisPolar(
                  driverController::getLeftX, driverController::getLeftY, 3)[0]
        };

    Command driveCommand =
        new DriveCommand(
            swerveDrive,
            visionSubsystem,
            // Translation in the X direction
            () -> driverLeftStick[0].getAsDouble() * 0.5,
            // Translation in the Y direction
            () -> driverLeftStick[1].getAsDouble() * 0.5,
            // Rotation
            () -> JoystickUtil.modifyAxis(driverController::getRightX, 3),
            // Robot relative
            () -> !driverController.rightBumper().getAsBoolean(),
            // Rotation speed
            driverController.rightStick(),
            // Drive speed for SWIM
            driverController.rightTrigger(),
            // Trench align
            driverController.leftBumper());

    // Sets the default command for the swerve drive to the drive command
    swerveDrive.setDefaultCommand(driveCommand);

    // driverController
    //     .y()
    //     .whileTrue(
    //         new FeedForwardCharacterization(
    //             swerveDrive,
    //             swerveDrive::runCharacterizationVoltage,
    //             swerveDrive::getCharacterizationVelocity));

    // Reset robot odometry based on the most recent vision pose measurement from april tags
    // This should be pressed when looking at an april tag
    driverController
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose())));

    // Resets the robot angle in the odometry, factors in which alliance the robot is on
    driverController
        .povRight()
        .onTrue(
            new InstantCommand(
                () ->
                    swerveDrive.resetEstimatedPose(
                        new Pose2d(
                            swerveDrive.getEstimatedPose().getX(),
                            swerveDrive.getEstimatedPose().getY(),
                            Rotation2d.fromDegrees(swerveDrive.getAllianceAngleOffset())))));

    driverController
        .b()
        .whileTrue(
            new DumperShootCommand(
                swerveDrive, shooterSubsystem, () -> operatorController.povDown().getAsBoolean()));
    driverController.a().whileTrue(new DefenseCommand(intakeSubsystem));
    // driverController.b().whileTrue(new ReverseSpindexerCommand(shooterSubsystem));
    driverController.leftTrigger().whileTrue(new ReverseKickerAndRollers(shooterSubsystem));

    // driverController
    //     .leftTrigger()
    //     .whileTrue(
    //         new HubLockCommand(swerveDrive, visionSubsystem, hoodSubsystem, turretSubsystem));
    // driverController.leftTrigger().whileTrue(new ReverseRollerFloor(shooterSubsystem));

    driverController
        .rightTrigger()
        .whileTrue(new ShootWhileMove(swerveDrive, shooterSubsystem, () -> false));
  }

  /** Configures the operator controller buttons and axes to control the robot */
  private void configureOperatorController() {
    // outake also runs kicker backwards
    operatorController
        .leftTrigger()
        .whileTrue(new OuttakeCommand(intakeSubsystem, shooterSubsystem));

    operatorController.y().whileTrue(new IntakePivotUpCommand(intakeSubsystem));
    operatorController.a().whileTrue(new IntakePivotDownCommand(intakeSubsystem));
    operatorController.x().whileTrue(new IntakePivotBounceLower(intakeSubsystem));
    // operatorController.b().whileTrue(new IntakePivotBounceHigher(intakeSubsystem));
    operatorController.b().whileTrue(new MoveIntakeUpCommand(intakeSubsystem));

    operatorController.rightTrigger().whileTrue(new IntakeCommand(intakeSubsystem));

    operatorController.povUp().whileTrue(new InstantCommand(() -> intakeSubsystem.zeroAngle()));
  }

  /** Checks the git status and records it to the log */
  private void checkGit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // This tells you if you have uncommitted changes in your project
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
  }

  /** Sets up the AdvantageKit logger */
  private void setupLogging() {
    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        // Gets data from network tables
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        {
          // Replaying a log, set up replay source
          setUseTiming(false); // Run as fast as possible
          String logPath = LogFileUtil.findReplayLog();
          Logger.setReplaySource(new WPILOGReader(logPath));
          Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
    }

    // Start AdvantageKit logger
    Logger.start();
  }

  @Override
  public void robotInit() {
    PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);
    powerDistribution.clearStickyFaults();
    powerDistribution.setSwitchableChannel(true);
    powerDistribution.close();
  }

  /** Sets up the subsystems based on the robot type */
  private void setupSubsystems() {
    switch (Constants.getRobot()) {
      case COMP_ROBOT -> {
        /* Real robot, instantiate hardware IO implementations */
        this.swerveDrive =
            new SwerveDrive(
                new PhysicalGyroPigeon(),
                new PhysicalModule(SwerveConstants.compModuleConfigs[0]),
                new PhysicalModule(SwerveConstants.compModuleConfigs[1]),
                new PhysicalModule(SwerveConstants.compModuleConfigs[2]),
                new PhysicalModule(SwerveConstants.compModuleConfigs[3]));
        this.visionSubsystem = new VisionSubsystem(new PhysicalVision() {}); // PhysicalVision
        this.shooterSubsystem = new ShooterSubsystem(new PhysicalShooter());
        this.intakeSubsystem = new IntakeSubsystem(new PhysicalIntake());
      }
      case DEV_ROBOT -> {
        /* Real robot, instantiate hardware IO implementations */
        // this.swerveDrive =
        //     new SwerveDrive(
        //         new PhysicalGyroNavX(),
        //         new PhysicalModule(SwerveConstants.devModuleConfigs[0]),
        //         new PhysicalModule(SwerveConstants.devModuleConfigs[1]),
        //         new PhysicalModule(SwerveConstants.devModuleConfigs[2]),
        //         new PhysicalModule(SwerveConstants.devModuleConfigs[3]));
        this.visionSubsystem = new VisionSubsystem(new VisionInterface() {});
        // this.shooterSubsystem = new ShooterSubsystem(new PhysicalShooter());
        // this.turretSubsystem = new TurretSubsystem(new PhysicalTurret());
        // THIS IS THE SYNTAX FOR WHATEVER SUBSYSTEMS ARE USED ^^
      }
      case SWERVE_ROBOT -> {
        /* Real robot, instantiate hardware IO implementations */
        // this.swerveDrive =
        //     new SwerveDrive(
        //         new PhysicalGyroNavX(),
        //         new PhysicalModule(SwerveConstants.aquilaModuleConfigs[0]),
        //         new PhysicalModule(SwerveConstants.aquilaModuleConfigs[1]),
        //         new PhysicalModule(SwerveConstants.aquilaModuleConfigs[2]),
        //         new PhysicalModule(SwerveConstants.aquilaModuleConfigs[3]));
        this.visionSubsystem = new VisionSubsystem(new VisionInterface() {});
        // this.shooterSubsystem = new ShooterSubsystem(new PhysicalShooter());
        // this.turretSubsystem = new TurretSubsystem(new PhysicalTurret());
      }

      case SIM_ROBOT -> {
        /* Sim robot, instantiate physics sim IO implementations */
        // this.simWorld = new SimWorld();
        // this.swerveDrive =
        //     new SwerveDrive(
        //         new SimulatedGyro(simWorld.robot().getDriveTrain().getGyro()),
        //         new SimulatedModule(0, simWorld.robot().getDriveTrain()),
        //         new SimulatedModule(1, simWorld.robot().getDriveTrain()),
        //         new SimulatedModule(2, simWorld.robot().getDriveTrain()),
        //         new SimulatedModule(3, simWorld.robot().getDriveTrain()));

        // this.visionSubsystem =
        //     new VisionSubsystem(new SimulatedVision(() -> simWorld.aprilTagSim()));
        // this.swerveDrive.resetEstimatedPose(new Pose2d(7, 4, new Rotation2d()));
        // this.elevatorSubsystem = new ElevatorSubsystem(new SimulatedElevator());
        // SYNTAX FOR SIM SUBSYSTEMS ^^
      }

      default -> {
        // this.visionSubsystem = new VisionSubsystem(new VisionInterface() {});
        /* Replayed robot, disable IO implementations */

        /* physics simulations are also not needed */
        // this.swerveDrive =
        //     new SwerveDrive(
        //         new GyroInterface() {},
        //         new ModuleInterface() {},
        //         new ModuleInterface() {},
        //         new ModuleInterface() {},
        //         new ModuleInterface() {});
        // this.elevatorSubsystem = new ElevatorSubsystem(new ElevatorInterface() {});
        // SYNTAX ^^
      }
    }
  }

  /** Sets up the auto commands */
  private void setupAuto() {
    this.autos =
        new Autos(
            this.swerveDrive, this.visionSubsystem, this.shooterSubsystem, this.intakeSubsystem);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
}
