/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ColorSensor;

public class RotationControl extends CommandBase {
  /**
   * Creates a new RotationControl.
   */
  private double ticks;
  
  public RotationControl() {

    addRequirements(ColorSensor.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
   /*
    m_sensor = sensor;
    SubsystemBase[] subsystems = {sensor};
    addRequirements((edu.wpi.first.wpilibj2.command.Subsystem[]) subsystems);
  */
    System.out.println("c test");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ColorSensor.getInstance().resetSensorPosition();
    System.out.println("start: " + ColorSensor.getInstance().getTicks());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   // if(ColorSensor.getInstance().getButton() == true){
    //  System.out.println("button state" +ColorSensor.getInstance().getButton());
      System.out.println(ColorSensor.getInstance().getTicks());
      ColorSensor.getInstance().spinWheel(0.3);
      ticks=ColorSensor.getInstance().getTicks();
    //}else{
      //ColorSensor.getInstance().spinWheel(0);
    //}
    //ColorSensor.getInstance().spinWheel(ColorSensor.getInstance().joy.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(ticks) > 4096 * 2)
    {
      System.out.println("finished");
      ColorSensor.getInstance().spinWheel(0);
      return true;  
    }

    else
    {
      return false;
    }
    
  }
}
