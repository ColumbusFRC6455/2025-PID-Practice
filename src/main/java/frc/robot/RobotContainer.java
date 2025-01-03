// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static Joystick joy1 = new Joystick(0);
  public final static CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  
  Command autoDrive = new SequentialCommandGroup(new DriveForwardCmd(driveSubsystem, 10 * Constants.DRIVE_MOTORS.ConversionFactor), new DriveForwardCmd(driveSubsystem, 24 * Constants.DRIVE_MOTORS.ConversionFactor));
  Command autoDrive2 = new SequentialCommandGroup(new DriveForwardCmd(driveSubsystem, 5 * Constants.DRIVE_MOTORS.ConversionFactor), new DriveForwardCmd(driveSubsystem, 10 * Constants.DRIVE_MOTORS.ConversionFactor));
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  

  

  public RobotContainer() {
    configureBindings();

    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem));

    
    autoChooser.setDefaultOption("Auto 1", autoDrive);
    autoChooser.addOption("Auto 2", autoDrive2);

    SmartDashboard.putData(autoChooser);
  }

  private void configureBindings() {
    /*
     * new Trigger(m_exampleSubsystem::exampleCondition)
     * .onTrue(new ExampleCommand(m_exampleSubsystem));
     */

    m_driverController.leftBumper().onTrue(new DriveForwardCmd(driveSubsystem, 10 * Constants.DRIVE_MOTORS.ConversionFactor));
    m_driverController.rightBumper().onTrue(new DriveForwardCmd(driveSubsystem, 24 * Constants.DRIVE_MOTORS.ConversionFactor));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
