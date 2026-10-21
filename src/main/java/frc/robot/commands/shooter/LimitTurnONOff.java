// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimitTurnONOff extends Command {
  /** Creates a new LimitTurnONOff. */
  SwerveConstants.ModuleConstants swerve;

  public LimitTurnONOff() {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.DRIVE_STATOR_LIMIT = 5.0;
    swerve.DRIVE_SUPPLY_LIMIT = 5.0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.DRIVE_STATOR_LIMIT = 60.0;
    swerve.DRIVE_SUPPLY_LIMIT = 55.0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
