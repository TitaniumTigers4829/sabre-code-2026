// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeCommand extends Command {
  /** Creates a new OuttakeCommand. */
  IntakeSubsystem intakeSubsystem;

  ShooterSubsystem shooterSubsystem;

  public OuttakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(intakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setIntakeAngle(IntakeConstants.PIVOT_DOWN_POSITION);
    shooterSubsystem.setRoller(-ShooterConstants.SPINDEXER_INTAKE_SPEED);
    if (intakeSubsystem.isIntakeDeployed()) {
      intakeSubsystem.outakeFuel();
    } else {
      intakeSubsystem.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setSpeed(0);
    shooterSubsystem.setRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
