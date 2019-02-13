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
import frc.robot.RobotMap;
import frc.robot.commands.LockLift;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  private static final double kP = 0.8;
  private static final double kI = 0.0;
  private static final double kD = 0.05;
  private double integral, previousError = 0.0;
  private double pidOutput = 0.0;

  private static final double PID_FINISHED_THRESHOLD = 0.5;  // in degrees
  private static ArrayList<Double> previousErrors = new ArrayList<>();

  private static WPI_VictorSPX liftMotor = new WPI_VictorSPX(RobotMap.liftMotor);

  private static Solenoid brake = new Solenoid(RobotMap.liftBrakeSolenoid);
  private static DoubleSolenoid upSolenoid = new DoubleSolenoid(RobotMap.liftUpSolenoidA, RobotMap.liftUpSolenoidB);
  private static DoubleSolenoid downSolenoid = new DoubleSolenoid(RobotMap.liftDownSolenoidA, RobotMap.liftDownSolenoidB);

  private static Counter leftEncoder = new Counter(new DigitalInput(RobotMap.liftLeftCounterPort));
  private static Counter rightEncoder = new Counter(new DigitalInput(RobotMap.liftRightCounterPort));
  private static final double COUNTER_ANGLE_PER_PULSE = 360 / 174.9;

  private double leftCounterReading = 0.0;
  private double rightCounterReading = 0.0;

  private static final double UPPER_LIMIT_READING = 50.0;

  private double setpoint = 0.0;

  private boolean isBraked = false;

  private static DigitalInput lowerLimitSwitch = new DigitalInput(RobotMap.liftLowerLimitSwitch);
  private static DigitalInput upperLimitSwitch = new DigitalInput(RobotMap.liftUpperLimitSwitch);

  public Lift() {
    leftEncoder.setDistancePerPulse(COUNTER_ANGLE_PER_PULSE);  // units are in degrees
    leftEncoder.setReverseDirection(true);

    rightEncoder.setDistancePerPulse(COUNTER_ANGLE_PER_PULSE);  // units are in degrees

    setSetpoint(0.0);
    setBrake(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new LockLift());
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
    if (isBraked && output != 0.0) {
      DriverStation.reportWarning("Lift is being moved while motor is enabled", false);
      return;
    }
    
    if (lowerLimitSwitch.get() && output < 0.0) {
      leftCounterReading = 0.0;
      rightCounterReading = 0.0;
      return;
    }
    if (upperLimitSwitch.get() && output > 0.0) {
      leftCounterReading = UPPER_LIMIT_READING;
      rightCounterReading = UPPER_LIMIT_READING;
      return;
    }

    if (output > 0.0) {
      upSolenoid.set(Value.kForward);
      downSolenoid.set(Value.kReverse);
    } else if (output < 0.0) {
      upSolenoid.set(Value.kReverse);
      downSolenoid.set(Value.kForward);
    } else {  // output == 0.0
      upSolenoid.set(Value.kForward);
      downSolenoid.set(Value.kForward);
    }

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
   * Enables or disables the disc brake. This should be set to true before
   * moving and set to false afterwards.
   * @param brakeOutput Whether the brake should be enabled
   */
  public void setBrake(boolean brakeOutput) {
    isBraked = brakeOutput;
    brake.set(brakeOutput);
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
    double error = setpoint - ((leftCounterReading + rightCounterReading) / 2); // Error = Target - Actual
    this.integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    double derivative = (error - this.previousError) / .02;
    double output = kP*error + kI*this.integral + kD*derivative;
    pidOutput = output;

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
}
