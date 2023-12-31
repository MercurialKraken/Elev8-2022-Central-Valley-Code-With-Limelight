// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class AutonomousFeederCommand extends CommandBase {
  private FeederSubsystem feederSubsystem;

  /** Creates a new AutonomousFeederCommand. */
  public AutonomousFeederCommand(FeederSubsystem feederSubsystem) {
    this.feederSubsystem = feederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.feederSubsystem.setFeederSpeed(0.22);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.feederSubsystem.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.autoEnabled)
    {return(false);}
    else
    return(RobotContainer.joyD.getRawButtonPressed(9));
  }
}
