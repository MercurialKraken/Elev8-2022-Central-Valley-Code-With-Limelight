// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class ShooterCommand extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  double rpm;
  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem shooterSubsystem, double rpm) {
    this.shooterSubsystem = shooterSubsystem;
    this.rpm = rpm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("setThisVelocity",shooterSubsystem.shoot );
    //SmartDashboard.putnumber("VEL", )
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.setSpeed(this.rpm);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.joyD.getRawButton(OIConstants.shooter_RB_ButtonNumber);
  }
}
