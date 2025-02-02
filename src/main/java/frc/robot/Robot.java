/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.utils.PressureSwitch;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  public static DriveTrain driveTrain;
  public static Lift lift;
  public static DualCamera dualCam;
  public static Hook hook;
  public static Compressor comp;
  public static PressureSwitch pressureSwitch;
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driveTrain = new DriveTrain();
    lift = new Lift();
    dualCam = new DualCamera();
    hook = new Hook();

    m_oi = new OI();

    comp = new Compressor();
    comp.setClosedLoopControl(false);
    comp.stop();
    pressureSwitch = new PressureSwitch();

    SmartDashboard.putNumber("kP", lift.kP);
    SmartDashboard.putNumber("kI", lift.kI);
    SmartDashboard.putNumber("kD", lift.kD);
    SmartDashboard.putNumber("kF", lift.kF);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("Lift left encoder reading", lift.leftCounterReading);
    // SmartDashboard.putNumber("Lift right encoder reading", lift.rightCounterReading);
    SmartDashboard.putBoolean("Lift at max height", lift.getUpperLimitSwitch());
    SmartDashboard.putBoolean("Lift at min height", !lift.getLowerLimitSwitch());
    SmartDashboard.putBoolean("Lift at setpoint", lift.isPidDone());

    // SmartDashboard.putNumber("Climber height", climber.getHeight());
    // SmartDashboard.putBoolean("Climber at max height", climber.getLimitSwitch());
    SmartDashboard.putNumber("Pressure", pressureSwitch.getPressure());
    SmartDashboard.putBoolean("BELOW REGULATED PRESSURE", pressureSwitch.getPressure() < 35.0);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    // lift.kP = SmartDashboard.getNumber("kP", lift.kP);
    // lift.kI = SmartDashboard.getNumber("kI", lift.kI);
    // lift.kD = SmartDashboard.getNumber("kD", lift.kD);
    // lift.kF = SmartDashboard.getNumber("kF", lift.kF);

    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
