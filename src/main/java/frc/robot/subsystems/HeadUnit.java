/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopHeadUnit;

/**
 * Add your docs here.
 */
public class HeadUnit extends Subsystem {
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.001;
  private double integral, previousError = 0.0;
  private double pidOutput = 0.0;

  // private static final double PID_FINISHED_THRESHOLD = 0.5;  // in degrees
  private static ArrayList<Double> previousErrors = new ArrayList<>();

  private static TalonSRX hookMotor = new TalonSRX(RobotMap.hookMotor);
  private static TalonSRX headTiltMotor = new TalonSRX(RobotMap.headTiltMotor);
  private static TalonSRX handMotor = new TalonSRX(RobotMap.handMotor);

  private static final double ENCODER_ANGLE_PER_PULSE = 360 / (4096 * 4);  // 4096 counts per revolution

  private static final double ENCODER_DOWN_ANGLE = 0.0;
  private static final double ENCODER_FORWARD_ANGLE = 90.0;
  private static final double ENCODER_UP_ANGLE = 180.0;

  private double setpoint = 0.0;
  // private HeadUnitPosition headUnitPosition = HeadUnitPosition.kDown;
  private Position headUnitPosition = Position.kHold;
  private Position hookPosition = Position.kHold;
  private Position handPosition = Position.kHold;

  public HeadUnit() {
    // headTiltMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    // headTiltMotor.setSelectedSensorPosition(0);
  }

  public enum HeadUnitPosition {
    kDown, kForward, kUp
  }

  public enum Position {
    kOpen, kClose, kHold
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new TeleopHeadUnit());
  }

  // public void setHeadTilt(HeadUnitPosition position) {
  //   headUnitPosition = position;
  //   switch (position) {
  //     case kDown:
  //     default:
  //       setpoint = ENCODER_DOWN_ANGLE;
  //       break;
  //     case kForward:
  //       setpoint = ENCODER_FORWARD_ANGLE;
  //       break;
  //     case kUp:
  //       setpoint = ENCODER_UP_ANGLE;
  //       break;
  //   }
  // }

  public void setHeadTilt(Position position) {
    // non-PID version
    headUnitPosition = position;
    switch (position) {
      case kHold:
      default:
        headTiltMotor.set(ControlMode.PercentOutput, 0.0);
        break;
      case kOpen:  // up
      headTiltMotor.set(ControlMode.PercentOutput, 1.0);
        break;
      case kClose:  // down
      headTiltMotor.set(ControlMode.PercentOutput, -1.0);
        break;
    }
  }

  public void pidTick() {
    double error = setpoint - getEncoder(); // Error = Target - Actual
    this.integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    double derivative = (error - this.previousError) / .02;
    double output = kP*error + kI*this.integral + kD*derivative;
    pidOutput = output;

    previousError = error;
    if (previousErrors.size() == 10) {
      previousErrors.remove(0);
    }
    previousErrors.add(Math.abs(error));

    headTiltMotor.set(ControlMode.PercentOutput, pidOutput);
  }

  /**
   * Defines the zero point of the absolute encoder value.
   * This should be at the bottom hard stop.
   */
  public void seedEncoder() {
    int pulseWidth = headTiltMotor.getSensorCollection().getPulseWidthPosition();
    pulseWidth = pulseWidth & 0xFFF;
    headTiltMotor.getSensorCollection().setPulseWidthPosition(pulseWidth, 20);
  }

  private double getEncoder() {
    return (headTiltMotor.getSensorCollection().getPulseWidthPosition() & 0xFFF) * ENCODER_ANGLE_PER_PULSE;
  }

  public void resetPid() {
    integral = 0.0;
    previousError = 0.0;
    pidOutput = 0.0;
    previousErrors = new ArrayList<>();
  }

  public void setHands(Position position) {
    handPosition = position;
    switch (position) {
      case kOpen:
        handMotor.set(ControlMode.PercentOutput, 0.2);
        break;
      case kClose:
        handMotor.set(ControlMode.PercentOutput, -0.2);
        break;
      case kHold:
      default:
        handMotor.set(ControlMode.PercentOutput, 0.0);
        break;
    }
  }

  public void setHook(Position position) {
    hookPosition = position;
    switch (position) {
      case kOpen:
        hookMotor.set(ControlMode.PercentOutput, 2.0 / 12.0);
        break;
      case kClose:
        hookMotor.set(ControlMode.PercentOutput, -1.0 / 12.0);
        break;
      case kHold:
      default:
        hookMotor.set(ControlMode.PercentOutput, 0.0);
        break;
    }
  }

  // public HeadUnitPosition getHeadUnitPosition() {
  //   return headUnitPosition;
  // }

  public Position getHeadUnitPosition() {
    return headUnitPosition;
  }

  public Position getHandPosition() {
    return handPosition;
  }
  
  public Position getHookPosition() {
    return hookPosition;
  }

  public void stopHeadUnit() {
    headTiltMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
