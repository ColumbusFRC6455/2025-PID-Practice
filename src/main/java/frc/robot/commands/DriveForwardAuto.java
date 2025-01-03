package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveForwardAuto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    double distance1;
    double distance2;
    DriveSubsystem subsystem;
  public DriveForwardAuto(DriveSubsystem subsystem, double distance1, double distance2) {
    this.subsystem = subsystem;
    this.distance1 = distance1;
    this.distance2 = distance2;
    


  }

}
