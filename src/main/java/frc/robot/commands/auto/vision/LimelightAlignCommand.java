// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class LimelightAlignCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  double target, ty, error;

  /** Creates a new LimelightAlignCommand. */
  public LimelightAlignCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = -9.5; //~42degs-54 = -12degs
    //SmartDashboard.putNumber("target", target);


    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0d);
    //SmartDashboard.putNumber("ty", ty);
    error = target - ty;
    //SmartDashboard.putNumber("error", error);
    driveSubsystem.setSpeeds(new double[] {0.05*error, 0.05*error});
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setSpeeds(new double[] {0,0});
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.autoEnabled)
    {return (error >= -0.5 && error <= 0.5);}
    else
    {return(RobotContainer.joyD.getRawButtonPressed(9));}
  }
}
