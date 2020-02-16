/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;

public class ColorControlCounter extends CommandBase {
  /**
   * Creates a new ColorControlCounter.
   */
  private String color1;  
  private String currentColor;

  private boolean c1;
  private int counter;

  public ColorControlCounter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ColorSensor.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override

  public void initialize() {
    color1 = ColorSensor.getInstance().matchColor();
    System.out.println("color 1");
    c1 = true;
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   currentColor = ColorSensor.getInstance().matchColor();
      System.out.println("current color it is detecting"+ currentColor);
    ColorSensor.getInstance().spinWheel(0.2);
    if(color1.equals(currentColor)){
      c1 = true;
      //System.out.println(c1);
      System.out.println("color1 is " + color1);
      System.out.println("current color is" + currentColor);
      System.out.println(color1.equals(currentColor));
    }
    if(c1 && !color1.equals(currentColor)){
      counter ++;
      System.out.println(counter);
      c1 = false;
      System.out.println(c1);
    }
    System.out.println("cureeeeeeeeent color is this:" + currentColor + " " + (!color1.equals(currentColor)));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter >= 7){
      ColorSensor.getInstance().spinWheel(0);
      return true;
    }
    else {
      return false;
    }
  }
}
