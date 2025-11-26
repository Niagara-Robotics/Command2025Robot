// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  //motor creaton
  private final TalonFX verticalMotor;
  private final TalonFX horizontalMotor;
  private final TalonFX rotateMotor;
  //creating PID for rotating the intake & creating a CANcoder that returns the motors current position
  private final CANcoder rotateEncoder;
  private final PIDController rotatePID;

  public IntakeSubsystem() {
    verticalMotor = new TalonFX(31);
    horizontalMotor = new TalonFX(32);
    rotateMotor = new TalonFX(30);
    rotateEncoder = new CANcoder(30);
    rotatePID = new PIDController(30, 0, 0.05); //FIX: Change p to 10 maybe, mess around with d also
  }
  
  public void SetVerticalIntake(double speed) {
    verticalMotor.set(speed);
  }

  public void SetHorizontalIntake(double speed) {
    horizontalMotor.set(speed);
  }

  public void RotateIntake(double intendedPosition) {
    double encoderValue = rotateEncoder.getPosition().getValueAsDouble();
    double speed = Constants.IntakeConstants.holdIntakeSpeed * Math.cos(encoderValue * Math.PI * 2);
    double PIDValue = rotatePID.calculate(rotateEncoder.getPosition().getValueAsDouble(), intendedPosition);
    rotateMotor.set(PIDValue + speed);
    SmartDashboard.putNumber("intakeSpeed", speed);
    SmartDashboard.putNumber("finalIntakeSpeed", PIDValue + speed);

  }

  public void TestMotor(double power) {
    rotateMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
