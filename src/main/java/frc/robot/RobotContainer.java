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
  public Joystick joy = new Joystick(0);
  public JoystickButton button;



  
  



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
    measureColors = new MeasureColors(colorSensor);
    System.out.println("d");
    rotational = new RotationControl(colorSensor);
    System.out.println("e");
    m_autoCommand = new RotationControl(colorSensor);;
    System.out.println("f");
    configureButtonBindings();
    System.out.println("g");

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
    button.whenPressed(new RotationControl(colorSensor));
    //Robot.m_robotContainer.sensor1.button.whileActive( new RotationControl());
    //button2 = new JoystickButton(joy,2);
    //button2.whenPressed(new PrintCommand("Command");
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
