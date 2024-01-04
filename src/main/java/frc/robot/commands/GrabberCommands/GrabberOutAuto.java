// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrabberCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GrabberOutAuto extends CommandBase {

  double endTime = 0;
  double duration = 0;

  /** Creates a new ElevatorUpCommand. */
  public GrabberOutAuto(double ms) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_grabberSubsystem);
    duration = ms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_grabberSubsystem.grabberOff();
    endTime = System.currentTimeMillis() + duration;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_grabberSubsystem.grabberOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_grabberSubsystem.grabberOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() >= endTime;
    /*return RobotContainer.m_elevatorSubsystem.currentElevatorPosition
    == frc.robot.Constants.elevatorHighAuto;*/
  }
}
