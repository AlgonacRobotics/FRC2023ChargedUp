// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElbowCommands.ElbowHalfLowerAutoCommand;
import frc.robot.commands.ElbowCommands.ElbowHighAutoCommand;
import frc.robot.commands.ElevatorCommands.ElevatorHighAutoCommand;
import frc.robot.commands.GrabberCommands.GrabberOutAuto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonRedRightCommandGroup extends SequentialCommandGroup {
  /** Creates a new ScoreHighCommandGroup. */
  public AutonRedRightCommandGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ElbowHalfLowerAutoCommand(1000),
        new ElevatorHighAutoCommand(3500),
        new ElbowHighAutoCommand(2000),
        Commands.waitSeconds(0.5),
        new GrabberOutAuto(1000),
        Commands.waitSeconds(0.5));
  
  }
}