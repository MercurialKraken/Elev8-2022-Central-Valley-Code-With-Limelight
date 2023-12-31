// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.pg.outer;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants.PGConstants;
import frc.robot.subsystems.climber.pg.OuterPGSubsystem;

public class OuterPGClimberMoveDistanceCommand extends CommandBase {
  /** Creates a new OuterPGClimberMoveDistanceCommand. */
  private OuterPGSubsystem outerPGSubsystem;
  private Supplier<Double> pg;
  private Supplier<Boolean> side;
  public OuterPGClimberMoveDistanceCommand(OuterPGSubsystem outerPGSubsystem, Supplier<Double> pg, Supplier<Boolean> side) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.outerPGSubsystem = outerPGSubsystem;
    this.pg = pg;
    this.side = side;
    addRequirements(this.outerPGSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pg = this.pg.get();
    boolean side = this.side.get();

    if (side)
      this.outerPGSubsystem.setPGOuterSpeed(-1 * pg);
    else
      this.outerPGSubsystem.setPGOuterSpeed(pg);
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
