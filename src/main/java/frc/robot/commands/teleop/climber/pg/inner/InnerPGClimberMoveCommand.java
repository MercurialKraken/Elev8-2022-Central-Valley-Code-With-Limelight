// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.pg.inner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.pg.InnerPGSubsystem;
import frc.robot.Constants.ClimberConstants.PGConstants;

public class InnerPGClimberMoveCommand extends CommandBase {
  InnerPGSubsystem innerPGSubsystem;
  double speed;


  /** Creates a new InnerPGClimberMoveCommand. */
  public InnerPGClimberMoveCommand(InnerPGSubsystem innerPGSubsystem, double speed) {
    this.innerPGSubsystem = innerPGSubsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(innerPGSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.innerPGSubsystem.setPGInnerSpeed(-1 * speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.innerPGSubsystem.setPGInnerSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
