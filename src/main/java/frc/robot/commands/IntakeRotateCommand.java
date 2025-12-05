// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeRotateCommand extends Command {

  CANcoder intakeRotation = new CANcoder(30);
  /** Creates a new IntakeRotateCommand. */
  private IntakeSubsystem intakeSubsystem;
  private double setpoint;
  private double speed;

  public IntakeRotateCommand(IntakeSubsystem intakeSubsystemInput, double setpointInput) {
    this.intakeSubsystem = intakeSubsystemInput;
    this.setpoint = setpointInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystemInput);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("Intake Encoder Position", intakeRotation.getPosition().getValueAsDouble());
    intakeSubsystem.RotateIntake(this.setpoint); // ~0.4 is up, ~0 is down
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.TestMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
