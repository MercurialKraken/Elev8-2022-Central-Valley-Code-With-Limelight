// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.pg.outer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.pg.OuterPGSubsystem;
import frc.robot.Constants.ClimberConstants.PGConstants;

public class OuterPGClimberMoveCommand extends CommandBase {
  private OuterPGSubsystem outerPGSubsystem;
  double speed;


  /** Creates a new OuterPGClimberMoveCommand. */
  public OuterPGClimberMoveCommand(OuterPGSubsystem outerPGSubsystem, double speed) {
    this.outerPGSubsystem = outerPGSubsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outerPGSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.outerPGSubsystem.setPGOuterSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.outerPGSubsystem.setPGOuterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
