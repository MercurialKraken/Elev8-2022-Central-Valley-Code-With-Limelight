// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.feeder.FeederByTimeCommand;
import frc.robot.commands.teleop.shooter.ShooterCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimeShot extends ParallelCommandGroup {
  /** Creates a new LimeShot. */
  public LimeShot(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //double time = 7;
    // double ty = RobotContainer
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0d);
    double dist = (2.6416-0.63) / Math.tan(Math.toRadians(ty+36)); //45 is limelight angle 2.6416 - ht to light
    double pow = 2590*dist/(2.15);

    if (true)
    {
    addCommands(new GoalAlignCommand(driveSubsystem));

    addCommands(new ShooterCommand(shooterSubsystem, 2590));

    //addCommands(new FeederByTimeCommand(feederSubsystem, 7));
    }

    
  }
}
