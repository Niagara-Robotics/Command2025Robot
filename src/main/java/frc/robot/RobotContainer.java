// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.IntakeMotorTestCommand;
import frc.robot.commands.IntakeRotateCommand;
import frc.robot.commands.RollerCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {  

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final Joystick driverStick = new Joystick(0);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    RollerCommand rollerCommand = new RollerCommand(intakeSubsystem, 0.5, 0.5);

    Trigger moveRollers = new Trigger(() -> driverStick.getRawButton(6));
    Trigger intakeDown = new Trigger(() -> driverStick.getRawButton(8));
    Trigger intakeUp = new Trigger(() -> driverStick.getRawButton(7));
    
    moveRollers.whileTrue(rollerCommand);
    intakeDown.whileTrue(new IntakeRotateCommand(intakeSubsystem, -0.05)); //RT
    intakeUp.whileTrue(new IntakeRotateCommand(intakeSubsystem, -0.35)); //LT
    // intakeUp.onTrue(new IntakeMotorTestCommand(intakeSubsystem, -0.03));


    // intakeSubsystem.setDefaultCommand(new IntakeRotateCommand(intakeSubsystem));
    // intakeSubsystem.setDefaultCommand(
    //   new IntakeCommand(intakeSubsystem, ()-> driverStick.getRawButton(6))
    // );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}
}
