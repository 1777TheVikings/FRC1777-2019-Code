/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.utils.SelfCheckError;

/**
 * Add your docs here.
 */
public class Lift extends PIDSubsystem {
  private static WPI_TalonSRX LeftLiftMotor = new WPI_TalonSRX(RobotMap.LeftLiftMotor);
  private static WPI_TalonSRX RightLiftMotor = new WPI_TalonSRX(RobotMap.RightLiftMotor);

  private static Solenoid brake = new Solenoid(RobotMap.liftBrakeSolenoid);
  private static DoubleSolenoid upSolenoid = new DoubleSolenoid(RobotMap.liftUpSolenoidA, RobotMap.liftUpSolenoidB);
  private static DoubleSolenoid downSolenoid = new DoubleSolenoid(RobotMap.liftDownSolenoidA, RobotMap.liftDownSolenoidB);

  private static Encoder encoder = new Encoder(RobotMap.liftEncoderAPort, RobotMap.liftEncoderBPort);
  private static int ENCODER_PPR = 2048;

  public Lift() {
    // Intert a subsystem name and PID values here
    super("Lift", 1, 2, 3);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    LeftLiftMotor.setInverted(true);
    RightLiftMotor.setInverted(true);

    encoder.setDistancePerPulse(360 / ENCODER_PPR);  // units are in degrees
    encoder.setMinRate(2);

    encoder.setDistancePerPulse(360 / ENCODER_PPR);  // units are in degrees
    encoder.setMinRate(2);
    encoder.setReverseDirection(true);

    setAbsoluteTolerance(3);
    setSetpoint(0.0);
    setBrake(true);
    disable();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void selfCheck() throws SelfCheckError {
    if (LeftLiftMotor.getFirmwareVersion() == 0)
      throw new SelfCheckError("Talon SRX with ID " + RobotMap.LeftLiftMotor + " is disconnected");
    if (RightLiftMotor.getFirmwareVersion() == 0)
      throw new SelfCheckError("Talon SRX with ID " + RobotMap.RightLiftMotor + " is disconnected");

  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return encoder.get();
  }

  @Override
  protected void usePIDOutput(double output) {
   LeftLiftMotor.pidWrite(output);
   RightLiftMotor.pidWrite(output);
  }

  public void tareEncoder() {
    encoder.reset();
  }

  public void setBrake(boolean enableBrake) {
    brake.set(enableBrake);
  }

  public void setSolenoids(Value value) {
    switch (value) {
      case kForward:
        upSolenoid.set(Value.kForward);
        downSolenoid.set(Value.kReverse);
        break;
      case kReverse:
        upSolenoid.set(Value.kReverse);
        downSolenoid.set(Value.kForward);
        break;
      case kOff:
      default:
        upSolenoid.set(Value.kForward);
        downSolenoid.set(Value.kForward);
        break;
    }
  }
}
