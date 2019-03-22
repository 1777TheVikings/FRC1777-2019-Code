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
import frc.robot.commands.hook.GrabHatch;
import frc.robot.commands.hook.ReleaseHatch;
import frc.robot.subsystems.Lift;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
  //creates controls for various aspects of robot
  public XboxController controller = new XboxController(0);
  public JoystickButton switchCameraButton = new JoystickButton(controller, 4); // y button
  public JoystickButton hatchGrabButton = new JoystickButton(controller, 3);  // x button
  public JoystickButton hatchReleaseButton = new JoystickButton(controller, 2);  // b button  

  // public Joystick secondaryController = new Joystick(1);
  // // TODO: Reflect actual port numbers
  // public JoystickButton groundButton = new JoystickButton(secondaryController, 1);
  // public JoystickButton level2Button = new JoystickButton(secondaryController, 2);
  // public JoystickButton level3Button = new JoystickButton(secondaryController, 3);
  // public JoystickButton hatchGrabButton = new JoystickButton(secondaryController, 4);
  // public JoystickButton hatchReleaseButton = new JoystickButton(secondaryController, 5);


  public OI()
  {
    //Command switchCameraCommand = new SwitchCamera();
    //switchCameraButton.whenPressed(switchCameraCommand);

    /**
     * whileHeld() will queue the command every tick by calling Command.start(), but
     * the scheduler only processes one instance of a Command subclass if multiple identical
     * ones are queued. Therefore, this will work as expected (command starts on button press
     * and receives interrupted() on release), but it may cause a slight bit of lag from
     * adding a command every tick.
     */

    //  groundButton.whenPressed(new MoveLift(Lift.GROUND_LEVEL_SETPOINT));
    //  level2Button.whenPressed(new MoveLift(Lift.LEVEL_2_SETPOINT));
    //  level3Button.whenPressed(new MoveLift(Lift.LEVEL_3_SETPOINT));

    hatchGrabButton.whenPressed(new GrabHatch());
    Command releaseHatchCommand = new ReleaseHatch();
    hatchReleaseButton.whileHeld(releaseHatchCommand);
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
    return controller.getTriggerAxis(Hand.kLeft) - controller.getTriggerAxis(Hand.kRight);
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
