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
  public static int frontLeftMotor = 2;  // Victor SP's
  public static int frontRightMotor = 0;
  public static int backLeftMotor = 3;
  public static int backRightMotor = 1;

  public static int pigeon = 1;  // Pigeon IMU
  
  // LIFT
  public static int liftMotor = 1;  // Victor SPX

  public static int liftUpSolenoidA = 0;  // Solenoids
  public static int liftUpSolenoidB = 1;
  public static int liftDownSolenoidA = 2;
  public static int liftDownSolenoidB = 3;

  public static int liftLeftCounterPort = 0;  // DIO
  public static int liftRightCounterPort = 1;

  public static int liftLowerLimitSwitch = 2;  // DIO
  public static int liftUpperLimitSwitch = 3;

  // HEAD UNIT
  public static int hookMotor = 1;  // Talon SRX
  public static int headTiltMotor = 2;
  public static int handMotor = 3;

  //Camera
  public static int backCamera = 0;  // USB ports
  public static int frontCamera = 1;

  // HOOK
  public static int hookExtendAPort = 4;  // Solenoids
  public static int hookExtendBPort = 5;
  public static int hookGrabAPort = 6;
  public static int hookGrabBPort = 7;

  // CLIMBER
  public static int climberBottomLeftMotor = 1;  // Talon SRX
  public static int climberTopLeftMotor = 2;
  public static int climberTopRightMotor = 3;
  public static int climberBottomRightMotor = 4;

  public static int climberSlideMotor = 0;  // PWM (Spark)

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
