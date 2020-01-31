/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.ColorSensor;

public class RotationControl extends CommandBase {
  /**
   * Creates a new RotationControl.
   */
  private final ColorSensor m_sensor;
  public RotationControl(ColorSensor sensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_sensor = sensor;
    SubsystemBase[] subsystems = {sensor};
    addRequirements((edu.wpi.first.wpilibj2.command.Subsystem[]) subsystems);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_robotContainer.sensor1.resetSensorPosition();
    System.out.println("start: " + Robot.m_robotContainer.sensor1.getTicks());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Robot.m_robotContainer.sensor1.getButton() == true){
      System.out.println("button state" +Robot.m_robotContainer.sensor1.getButton());
      System.out.println(Robot.m_robotContainer.sensor1.getTicks());
      Robot.m_robotContainer.sensor1.spinWheel(0.3);
    }else{
      Robot.m_robotContainer.sensor1.spinWheel(0);
    }
    //Robot.m_robotContainer.sensor1.spinWheel(Robot.m_robotContainer.sensor1.joy.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(Robot.m_robotContainer.sensor1.getTicks()) > 4096 * 10){
      System.out.println("finished");
      return true;
    }

    else{
      return false;
    }
    
  }
}
