/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // DRIVE TRAIN
  public static int frontLeftMotor = 0;  // Victor SP's
  public static int frontRightMotor = 1;
  public static int backLeftMotor = 2;
  public static int backRightMotor = 3;

  public static int pigeon = 1;  // Pigeon IMU
  
  // LIFT
  public static int liftMotor = 1;  // Victor SPX

  public static int liftBrakeSolenoid = 0;  // Solenoids
  public static int liftUpSolenoidA = 1;
  public static int liftUpSolenoidB = 2;
  public static int liftDownSolenoidA = 3;
  public static int liftDownSolenoidB = 4;

  public static int liftLeftCounterPort = 0;  // DIO
  public static int liftRightCounterPort = 1;

  public static int liftLowerLimitSwitch = 2;  // DIO
  public static int liftUpperLimitSwitch = 3;

  //Camera
  public static int backCamera = 0;  // USB ports
  public static int frontCamera = 1;


  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
