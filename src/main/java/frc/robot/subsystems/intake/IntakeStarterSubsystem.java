// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeStarterSubsystem extends SubsystemBase {
  private Servo starterServo1, starterServo2;
  //private WPI_TalonSRX starterServoFTCMotor;

  /** Creates a new IntakeStarterSubsystem. */
  public IntakeStarterSubsystem() {
    starterServo1 = new Servo(1);
    //this.starterServoFTCMotor = new WPI_TalonSRX(45);
    starterServo1.setAngle(100);
    starterServo2 = new Servo(2);
    starterServo2.setAngle(100);
    //SmartDashboard.putNumber("servoi", this.starterServo1.getAngle());
    //this.starterServoFTCMotor.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("servo", this.starterServo1.getAngle());
    // this.starterServo1.setAngle(40);
    // this.starterServo2.setAngle(160);
  }

  public void setSpeed(double pg) {
    // this.starterServoFTCMotor.set(TalonSRXControlMode.PercentOutput, pg);
  }

  public void setAngle() {
    this.starterServo1.setAngle(35);
    this.starterServo2.setAngle(165);
  }

  public void setFeederSpeed(double feederSpeed) {
    // double speedmotor = this.starterServoFTCMotor.get();
    // if (Math.abs(speedmotor) < 0.05) {
    //   double now = Timer.getFPGATimestamp();
    //   while (now < 0.6) {
    //     if (now == 0.2)
    //       this.starterServoFTCMotor.set(feederSpeed / 3);
    //     if (now == 0.4)
    //       this.starterServoFTCMotor.set(feederSpeed / 2);
    //   }
    //   this.starterServoFTCMotor.set(feederSpeed);
    //   // this.stopper.setAngle(FeederConstants.positionAngle);
    //   // System.out.println("Timer : " + now);
    // } else {
    //   this.starterServoFTCMotor.set(0);
    //   // this.stopper.setAngle(FeederConstants.initialAngle);

    }
  }

