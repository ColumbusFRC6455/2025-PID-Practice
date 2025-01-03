// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DRIVE_MOTORS;

public class DriveSubsystem extends SubsystemBase {

  public final static CANSparkMax driveMotorLeft0 = new CANSparkMax(Constants.DRIVE_MOTORS.leftMotor0,
      MotorType.kBrushless);
  public final static CANSparkMax driveMotorLeft1 = new CANSparkMax(Constants.DRIVE_MOTORS.leftMotor1,
      MotorType.kBrushless);
  public final static CANSparkMax driveMotorRight0 = new CANSparkMax(Constants.DRIVE_MOTORS.rightMotor0,
      MotorType.kBrushless);
  public final static CANSparkMax driveMotorRight1 = new CANSparkMax(Constants.DRIVE_MOTORS.rightMotor1,
      MotorType.kBrushless);

  public static PIDController DrivePIDl;
  public static PIDController DrivePIDr;
  double drivePowerL;
  double drivePowerR;

  final static DifferentialDrive m_drive = new DifferentialDrive(driveMotorLeft0, driveMotorRight0);

  final static RelativeEncoder m_leftEncoder = driveMotorLeft0.getEncoder();
  final static RelativeEncoder m_rightEncoder = driveMotorRight0.getEncoder();

  public double getEncoderInches() {
    return (m_leftEncoder.getPosition() + -m_rightEncoder.getPosition()) / 2;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive encoder value", getEncoderInches());
  } 

  public void setMotors(double speed) {
    driveMotorLeft0.set(speed);
    driveMotorRight0.set(-speed);
  }

  public static void arcadeDrive(double x, double y) {
    m_drive.setSafetyEnabled(false);
    m_drive.arcadeDrive(x *.8 , y);
  }

  public static void resetEncoders(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    
  }

  public DriveSubsystem() {
    m_leftEncoder.setPositionConversionFactor(DRIVE_MOTORS.ConversionFactor);
    m_rightEncoder.setPositionConversionFactor(DRIVE_MOTORS.ConversionFactor);
    
  }

}
