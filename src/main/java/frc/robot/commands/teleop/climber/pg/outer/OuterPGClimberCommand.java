// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.climber.pg.outer;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants.PGConstants;
import frc.robot.subsystems.climber.pg.OuterPGSubsystem;

public class OuterPGClimberCommand extends CommandBase {
  private OuterPGSubsystem outerPGSubsystem;
  private Supplier<Double> pg;
  private Supplier<Boolean> side;

  /** Creates a new OuterPGClimberCommand. */
  public OuterPGClimberCommand(OuterPGSubsystem outerPGSubsystem, Supplier<Double> pg, Supplier<Boolean> side) {
    this.outerPGSubsystem = outerPGSubsystem;
    this.pg = pg;
    this.side = side;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.outerPGSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pg = this.pg.get();
    boolean side = this.side.get();

    this.outerPGSubsystem.setPGOuterSpeed(-1 * this.pg.get() * PGConstants.speedMultiplier);


    // if (side)
    //   this.outerPGSubsystem.setPGOuterSpeed(pg, PGConstants.speedMultiplier);
    // else
    //   this.outerPGSubsystem.setPGOuterSpeed(-1 * pg, PGConstants.speedMultiplier);
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
