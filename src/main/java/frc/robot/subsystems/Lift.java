/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.LockLift;
import frc.robot.commands.TeleopLift;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  public double kP = 0.1;
  public double kI = 0.0;
  public double kD = 0.0;
  public double kF = 0.0;
  private double integral, previousError = 0.0;
  private double pidOutput = 0.0;

  private static final double PID_FINISHED_THRESHOLD = 0.5;  // in degrees
  private static ArrayList<Double> previousErrors = new ArrayList<>();

  private static WPI_VictorSPX liftMotor = new WPI_VictorSPX(RobotMap.liftMotor);

  private static DoubleSolenoid solenoid = new DoubleSolenoid(RobotMap.liftSolenoidA, RobotMap.liftSolenoidB);

  private static Counter leftEncoder = new Counter(new DigitalInput(RobotMap.liftLeftCounterPort));
  private static Counter rightEncoder = new Counter(new DigitalInput(RobotMap.liftRightCounterPort));
  private static final double COUNTER_ROTATIONS_PER_PULSE = 1 / 174.9;

  public double leftCounterReading = 0.0;
  public double rightCounterReading = 0.0;

  // TODO: Set this to the actual value
  private static final double UPPER_LIMIT_READING = 50.0;

  private double setpoint = 0.0;

  private static DigitalInput lowerLimitSwitch = new DigitalInput(RobotMap.liftLowerLimitSwitch);
  private static DigitalInput upperLimitSwitch = new DigitalInput(RobotMap.liftUpperLimitSwitch);

  // TODO: Make these not 0.0
  public static final double GROUND_LEVEL_SETPOINT = 0.0;
  public static final double LEVEL_2_SETPOINT = 0.0;
  public static final double LEVEL_3_SETPOINT = 0.0;

  // public static final double ARM_LENGTH = 28.0;
  // public static final double ARM_WEIGHT = 40.0;
  // public static final double PISTON_BORE = 1.25;
  // public static final double SPROCKET_GEARING = 32.0 / 20.0;
  // public static final double MIN_ANGLE = -40;  // in degrees

  public Lift() {
    // leftEncoder.setDistancePerPulse(COUNTER_ROTATIONS_PER_PULSE);  // units are in degrees
    leftEncoder.setReverseDirection(true);

    // rightEncoder.setDistancePerPulse(COUNTER_ROTATIONS_PER_PULSE);  // units are in degrees

    solenoid.set(Value.kForward);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new TeleopLift());
    // setDefaultCommand(new LockLift());
  }

  /**
   * This method manually controls the motor outputs. It also obeys the limit switches
   * and sets the solenoids appropriately. A warning will be issued if an output other
   * than 0.0 is used while the brake is enabled.
   * 
   * This is used internally by pidTick(), but it can also be used for manually
   * controlling the lift with a joystick. The former method is strongly preferred.
   * 
   * @param output The speed to set the motors to; must be in range [-1, 1]
   */
  public void useMotors(double output) {
    // if (!getLowerLimitSwitch() && output < 0.0) {
    //   leftCounterReading = 0.0;
    //   rightCounterReading = 0.0;
    //   liftMotor.set(0.0);
    //   return;
    // }
    if (getUpperLimitSwitch() && output > 0.0) {
      // TODO: Uncomment these after setting UPPER_LIMIT_READING
      // leftCounterReading = UPPER_LIMIT_READING;
      // rightCounterReading = UPPER_LIMIT_READING;
      liftMotor.set(0.0);
      return;
    }
    
    if (Robot.m_oi.getLiftDown())
      solenoid.set(Value.kReverse);
    else
      solenoid.set(Value.kForward);

    liftMotor.set(output);
  }

  /**
   * This sets the setpoint used by the PID loop.
   * @param setpoint The setpoint to move towards, in degrees. 0.0 degrees
   * is at the lift's resting point.
   */
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  /**
   * Updates the counters built into the Bosch seat motors and turns their
   * values into something useful. This should be called every tick and
   * before calling pidTick().
   */
  public void updateCounters() {
    if (pidOutput > 0.0) {
      leftCounterReading += leftEncoder.get();
      rightCounterReading += rightEncoder.get();
    } else {
      leftCounterReading -= leftEncoder.get();
      rightCounterReading -= rightEncoder.get();
    }

    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Calculates the motor outputs based on the setpoint. This should be
   * called every tick when using PID control.
   */
  public void pidTick() {
    double error = setpoint - getCounterReading(); // Error = Target - Actual
    this.integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    double derivative = (error - this.previousError) / .02;
    double output = kP*error + kI*this.integral + kD*derivative + kF;
    pidOutput = output;

    previousError = error;
    if (previousErrors.size() == 10) {
      previousErrors.remove(0);
    }
    previousErrors.add(Math.abs(error));

    useMotors(output);
  }

  public boolean isPidDone() {
    double sum = 0.0;
    for (Double error : previousErrors)
      sum += error;
    
    return (sum / previousErrors.size()) <= PID_FINISHED_THRESHOLD;
  }

  /**
   * Resets all of the values used internally by the PID controller.
   * This should be called when PID motions are stopped (e.g. when
   * braked).
   */
  public void pidReset() {
    integral = 0.0;
    previousError = 0.0;
    pidOutput = 0.0;
    previousErrors = new ArrayList<>();
  }

  public double getCounterReading() {
    return (leftCounterReading + rightCounterReading) / 2;
  }

  public boolean getLowerLimitSwitch() {
    return lowerLimitSwitch.get();
  }

  public boolean getUpperLimitSwitch() {
    return upperLimitSwitch.get();
  }

  // public double getMotorHoldingOutput() {
  //   double singlePistonForce = Math.min(Robot.pressureSwitch.getPressure() + 5.0, 30.0) * Math.PI * Math.pow(PISTON_BORE, 2) / 4;
  //   double gravityTorque = 2.0 * ARM_LENGTH * (ARM_WEIGHT - (2 * singlePistonForce)) * Math.cos(Math.toRadians(getCounterReading()));
  //   return gravityTorque / ((194.4 * SPROCKET_GEARING) * 2.0);
  // }
}
