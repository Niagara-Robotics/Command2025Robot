// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeMotorTestCommand extends Command {
  
  private IntakeSubsystem intakeSubsystem;
  private double speed;
  CANcoder intakeRotation = new CANcoder(30);


  /** Creates a new IntakeMotorTestCommand. */
  public IntakeMotorTestCommand(IntakeSubsystem intakeSubsystemInput, double speedInput) {
    this.intakeSubsystem = intakeSubsystemInput;
    this.speed = speedInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double encoderValue = intakeRotation.getPosition().getValueAsDouble();
    speed = Constants.IntakeConstants.holdIntakeSpeed * Math.cos(encoderValue * Math.PI * 2);
    intakeSubsystem.TestMotor(speed);
    SmartDashboard.putNumber("intakeSpeed", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
