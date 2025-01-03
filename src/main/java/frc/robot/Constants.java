// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DRIVE_MOTORS {
    public static final int leftMotor0 = 2;
    public static final int leftMotor1 = 4;
    public static final int rightMotor0 = 1;
    public static final int rightMotor1 = 3;
    //drive base conversion TANK
  public static final double ConversionFactor = (6 * Math.PI);
  public static final double ConversionFactorInches =  10/7.5;
  
  
      
  }
  public static class PID {
  public static final double kP = .01;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
  }

public static final String INTAKE_MOTORS = null;
}
