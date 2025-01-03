package frc.robot.commands;

import frc.robot.Constants.PID;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForwardCmd extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private DriveSubsystem driveSub = new DriveSubsystem();
  private double setpoint;
  private double encoderSetpoint;
  private PIDController pidController;

  public DriveForwardCmd(DriveSubsystem driveSubsystem, double setpoint) {
    this.driveSub = driveSubsystem;
    this.setpoint = setpoint;
    this.pidController = new PIDController(PID.kP, PID.kI, PID.kD);
    pidController.setSetpoint(setpoint);
    pidController.setTolerance(2.23);
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("DriveCommand started!");
    pidController.reset();
    DriveSubsystem.resetEncoders();
    encoderSetpoint = driveSub.getEncoderInches() + setpoint;
  }

  @Override
  public void execute() {
    double speed = pidController.calculate(driveSub.getEncoderInches());
    driveSub.setMotors(speed * .2);

  }

  @Override
  public void end(boolean interrupted) {
      driveSub.setMotors(0);
      System.out.println("Drive Command Ended" );
      System.out.println(driveSub.getEncoderInches());
    
  }

  @Override
  public boolean isFinished() {
    if (pidController.atSetpoint()){
      return true;
    } else{
      return false;
    }

  }
}
