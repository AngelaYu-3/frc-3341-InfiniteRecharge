/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.MeasureColors;
import frc.robot.commands.RotationControl;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ColorControl;
import frc.robot.commands.ColorControlCounter;
//import edu.wpi.first.wpilibj.util.Color;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.commands.ExampleCommand;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem;
  public ColorSensor colorSensor;
  public MeasureColors measureColors;
  public RotationControl rotational;
  private final RotationControl m_autoCommand;
  public ColorControlCounter colorControlCounter;
  public ColorControl colorControl;
  public Joystick joy = new Joystick(0);
  public JoystickButton button;

  public JoystickButton buttonRed;
  public JoystickButton buttonBlue;
  public JoystickButton buttonGreen;
  public JoystickButton buttonYellow;
  public JoystickButton colorCountingControler;



  
  



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    System.out.println("a");
    m_exampleSubsystem = new ExampleSubsystem();
    System.out.println("b");
    colorSensor = new ColorSensor();
    System.out.println("c");
    measureColors = new MeasureColors();
    System.out.println("d");
    rotational = new RotationControl();
    System.out.println("e");
    m_autoCommand = new RotationControl();
    System.out.println("f");
    colorControl = new ColorControl(0, colorSensor);
    colorControlCounter = new ColorControlCounter();
    System.out.println("g");
    configureButtonBindings();
    System.out.println("h");

    //CommandScheduler.getInstance().schedule(measureColors);
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    button = new JoystickButton(joy, 1);
    buttonRed = new JoystickButton(joy, 2);
    buttonBlue = new JoystickButton(joy, 3);
    buttonGreen = new JoystickButton(joy,4);
    buttonYellow = new JoystickButton(joy,5);
    colorCountingControler = new JoystickButton(joy, 7);
    button.whenPressed(new RotationControl());
    buttonRed.whenPressed(new ColorControl(1, colorSensor));
    buttonBlue.whenPressed(new ColorControl(2, colorSensor));
    buttonGreen.whenPressed(new ColorControl(3, colorSensor)); 
    buttonYellow.whenPressed(new ColorControl(4, colorSensor));
    colorCountingControler.whenPressed(new ColorControlCounter());
    //Robot.m_robotContainer.sensor1.button.whileActive( new RotationControl());
    //button2 = new JoystickButton(joy,2);
    //button2.whenPressed(new PrintCommand("Command"
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
