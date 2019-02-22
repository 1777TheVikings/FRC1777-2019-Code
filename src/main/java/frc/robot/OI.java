/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.MoveLift;
import frc.robot.commands.SwitchCamera;
import frc.robot.commands.auto_alignment.TurnToTarget;
import frc.robot.subsystems.Lift;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
import frc.robot.subsystems.HeadUnit.HeadUnitPosition;
import frc.robot.subsystems.HeadUnit.Position;

public class OI {
  //creates controls for various aspects of robot
  public XboxController controller = new XboxController(0);
  public JoystickButton autoAlignButton = new JoystickButton(controller, 3);  // X button
  public JoystickButton switchCameraButton = new JoystickButton (controller, 4); // y button

  public Joystick secondaryController = new Joystick(1);
  public JoystickButton groundButton = new JoystickButton(secondaryController, 1);
  public JoystickButton level1CargoButton = new JoystickButton(secondaryController, 2);
  public JoystickButton level2HatchButton = new JoystickButton(secondaryController, 3);
  public JoystickButton level2CargoButton = new JoystickButton(secondaryController, 4);
  public JoystickButton level3HatchButton = new JoystickButton(secondaryController, 5);
  public JoystickButton level3CargoButton = new JoystickButton(secondaryController, 6);

  public OI()
  {
    // Command switchCameraCommand = new SwitchCamera();
    // switchCameraButton.whenPressed(switchCameraCommand);
    // Command autoAlignCommand = new TurnToTarget();
    // autoAlignButton.whileHeld(autoAlignCommand);
    /**
     * The above line will queue the command every tick by calling Command.start(), but
     * the scheduler only processes one instance of a Command subclass if multiple identical
     * ones are queued. Therefore, this will work as expected (command starts on button press
     * and receives interrupted() on release), but it may cause a slight bit of lag from
     * adding a command every tick.
     */

    //  groundButton.whenPressed(new MoveLift(Lift.GROUND_LEVEL_SETPOINT));
    //  level1CargoButton.whenPressed(new MoveLift(Lift.LEVEL_1_CARGO_SETPOINT));
    //  level2HatchButton.whenPressed(new MoveLift(Lift.LEVEL_2_HATCH_SETPOINT));
    //  level2CargoButton.whenPressed(new MoveLift(Lift.LEVEL_2_CARGO_SETPOINT));
    //  level3HatchButton.whenPressed(new MoveLift(Lift.LEVEL_3_HATCH_SETPOINT));
    //  level3CargoButton.whenPressed(new MoveLift(Lift.LEVEL_3_CARGO_SETPOINT));
  }

  public double getDriveY() {    
    return controller.getY(Hand.kLeft);
  }

  public double getDriveX() {
    return controller.getX(Hand.kLeft);
  }
	
  public double getDriveTwist() {
    return controller.getX(Hand.kRight);
  }
  
  public double getLift() {
    return controller.getY(Hand.kRight);
  }

  // public HeadUnitPosition getHeadUnitTilt() {
  //   if (secondaryController.getRawButton(7))
  //     return HeadUnitPosition.kDown;
  //   else if (secondaryController.getRawButton(8))
  //     return HeadUnitPosition.kForward;
  //   else if (secondaryController.getRawButton(9))
  //     return HeadUnitPosition.kUp;
  //   else
  //     return Robot.headUnit.getHeadUnitPosition();
  // }

  public Position getHeadUnitTilt() {
    if (secondaryController.getRawButton(7))
      return Position.kClose;
    else if (secondaryController.getRawButton(8))
      return Position.kHold;
    else if (secondaryController.getRawButton(9))
      return Position.kOpen;
    else
      return Position.kHold;
  }

  public Position getHandPosition() {
    if (secondaryController.getRawButton(10))
      return Position.kClose;
    else if (secondaryController.getRawButton(11))
      return Position.kOpen;
    else
      return Position.kHold;
  }

  public Position getHookPosition() {
    if (secondaryController.getRawButton(12))
      return Position.kClose;
    // joysticks only support 12 buttons, so the 13th one is wired to axis 0
    else if (secondaryController.getRawAxis(0) < 0.0)
      return Position.kOpen;
    else
      return Position.kHold;
  }

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
