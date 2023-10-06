// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class GoalSeekingCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  double tv;
  /** Creates a new GoalSeekingCommand. */
  public GoalSeekingCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
    driveSubsystem.drive(0.4, -0.4); //can change speeds if there is overshoot
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.tv == 1.0;
  }
}
