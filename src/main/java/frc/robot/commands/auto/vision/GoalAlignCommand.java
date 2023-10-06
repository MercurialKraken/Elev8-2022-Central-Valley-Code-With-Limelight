// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class GoalAlignCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  double error, tx, turn;
  /** Creates a new GoalAlignCommand. */
  public GoalAlignCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //DriveSubsystem.navx.reset();
    this.tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //this.turn = DriveSubsystem.navx.getYaw() + this.tx;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) - 5;
    this.driveSubsystem.setSpeeds(new double[] {error*0.025, error*-0.025});
    //this.driveSubsystem.setSpeeds(this.driveSubsystem.speedcontrolforanglecorrect(-error));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.setSpeeds(new double[] { 0, 0 });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.autoEnabled)
    {return error <= 2 && error >= -2;}
    else
    {return (RobotContainer.joyD.getRawButtonPressed(9));}
  }
}
