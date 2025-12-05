// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

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
  private final Slot0Configs slotConfigs;
  //creating PID for rotating the intake & creating a CANcoder that returns the motors current position
  private final CANcoder rotateEncoder;
  private final PIDController rotatePID;
  private final PositionDutyCycle positionRequest;
  // private final TalonFXConfiguration rotateConfig;
  private final FeedbackConfigs feedbackConfigs;

  public IntakeSubsystem() {
    verticalMotor = new TalonFX(31);
    horizontalMotor = new TalonFX(32);
    rotateMotor = new TalonFX(30);
    slotConfigs = new Slot0Configs();
    feedbackConfigs = new FeedbackConfigs();
    // rotateConfig = new TalonFXConfiguration();
    rotateEncoder = new CANcoder(30);
    rotatePID = new PIDController(100, 0, 0.1); //FIX: Change p to 10 maybe, mess around with d also (P was 30 d was 0.05)
    positionRequest = new PositionDutyCycle(0);
    System.out.println("HELP");

    feedbackConfigs.FeedbackRemoteSensorID = 30;
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    
    // rotateConfig.Slot0.kP = 60; //FIX: change back to 120?
    // rotateConfig.Slot0.kI = 0;
    // rotateConfig.Slot0.kD = 0.1;
    // rotateConfig.Slot0.kS = 0.2;
    // rotateConfig.Slot0.kV = 1.6;
    // rotateConfig.Slot0.kG = 0.3;
    // rotateConfig.Slot0.kP = 0.3;
    // rotateConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    slotConfigs.kP = 60;
    slotConfigs.kI = 0;
    slotConfigs.kD = 0.2;
    slotConfigs.kS = 0.2;
    slotConfigs.kV = 0;
    slotConfigs.kG = -0.03;
    slotConfigs.kA = 0.1;
    slotConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    slotConfigs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    rotateMotor.getConfigurator().apply(slotConfigs);
    rotateMotor.getConfigurator().apply(feedbackConfigs);
  }
  
  public void SetVerticalIntake(double speed) {
    verticalMotor.set(speed);
  }

  public void SetHorizontalIntake(double speed) {
    horizontalMotor.set(speed);
  }

  public void RotateIntake(double intendedPosition) {
    // double encoderValue = rotateEncoder.getPosition().getValueAsDouble();
    // double speed = Constants.IntakeConstants.holdIntakeSpeed * Math.cos(encoderValue * Math.PI * 2);
    // double PIDValue = rotatePID.calculate(rotateEncoder.getPosition().getValueAsDouble(), intendedPosition);
    // rotateMotor.set(PIDValue + speed);
    // SmartDashboard.putNumber("intakeSpeed", speed);
    // SmartDashboard.putNumber("finalIntakeSpeed", PIDValue + speed);
    rotateMotor.setControl(positionRequest.withPosition(intendedPosition));
  
  }

  public void TestMotor(double power) {
    rotateMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
