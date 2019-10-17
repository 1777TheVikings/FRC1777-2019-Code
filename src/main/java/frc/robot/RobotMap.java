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

  public static int liftSolenoidA = 0;  // Solenoids
  public static int liftSolenoidB = 1;

  public static int liftLeftCounterPort = 0;  // DIO
  public static int liftRightCounterPort = 1;

  public static int liftLowerLimitSwitch = 3;  // DIO
  public static int liftUpperLimitSwitch = 2;

  //Camera
  public static int backCamera = 1;  // USB ports
  public static int frontCamera = 0;

  // HOOK
  public static int hookPusherAPort = 2;  // Solenoids
  public static int hookPusherBPort = 3;
  public static int hookVacuumAPort = 4;
  public static int hookVacuumBPort = 5;

  // CLIMBER
  public static int climberBottomLeftMotor = 1;  // Talon SRX
  public static int climberTopLeftMotor = 2;
  public static int climberTopRightMotor = 3;
  public static int climberBottomRightMotor = 4;

  public static int climberSlideMotor = 2;  // Victor SPX

  public static int climberLimitSwitch = 4;  // DIO
  public static int climberDistanceSensor = 5;

  // OTHER
  public static int lightDriveAPort = 4;  // PWM
  public static int lightDriveBPort = 5;

  public static int pressureSwitchIn = 0;  // Analog In

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
