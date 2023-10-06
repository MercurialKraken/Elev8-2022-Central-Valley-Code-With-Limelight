// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.routine;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.drive.tester.DriveBySecondCoordinateCommand;
import frc.robot.commands.auto.drive.tester.DriveToACoordinateCommand;
import frc.robot.commands.auto.feeder.AutonomousFeederCommand;
import frc.robot.commands.auto.vision.GoalAlignCommand;
import frc.robot.commands.auto.vision.LimeDance;
import frc.robot.commands.auto.vision.LimelightAlignCommand;
import frc.robot.commands.teleop.feeder.FeederCommand;
import frc.robot.commands.teleop.shooter.ShooterCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.servo.ServoFeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CompleteMotionGroupCommand extends SequentialCommandGroup {
  public double FIRST_X = 1.5; // 1.5
  public double FIRST_Y = 0.0;
  public double SECOND_X = 0.55;
  public double SECOND_Y = 0;
  public double ANGLE_SECOND = 180;

  /** Creates a new CompleteMotionGroupCommand. */
  public CompleteMotionGroupCommand(FeederSubsystem feederSubsystem, DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ServoFeederSubsystem servoFeederSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveToACoordinateCommand(driveSubsystem, servoFeederSubsystem, FIRST_X, FIRST_Y));
    addCommands(new WaitCommand(0.75));
    addCommands(new DriveBySecondCoordinateCommand(driveSubsystem, servoFeederSubsystem, ANGLE_SECOND));
    addCommands(new DriveToACoordinateCommand(driveSubsystem, servoFeederSubsystem, SECOND_X, SECOND_Y));
    addCommands(new DriveBySecondCoordinateCommand(driveSubsystem, servoFeederSubsystem, ANGLE_SECOND));
    addCommands(new LimeDance(feederSubsystem, driveSubsystem));
    // addCommands(new LimelightAlignCommand(driveSubsystem));
    // addCommands(new GoalAlignCommand(driveSubsystem));
    // addCommands(new AutonomousFeederCommand(feederSubsystem));
    // addCommands(new ShooterCommand(shooterSubsystem, 0));
    addCommands(new FeederCommand(feederSubsystem));
    //addCommands(new AutonomousFeederCommand(feederSubsystem));
    // addCommands(new AutonomousFeederCommand(feederSubsystem));

  }
}
