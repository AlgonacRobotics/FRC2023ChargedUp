// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElbowCommands.ElbowHomeAutoCommand;
import frc.robot.commands.ElevatorCommands.ElevatorHomeAutoCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHomeMiddleBalanceCommandGroup extends SequentialCommandGroup {
  /** Creates a new HomeCommandGroup. */
  public ScoreHomeMiddleBalanceCommandGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ElevatorHomeAutoCommand(1000), new ElbowHomeAutoCommand(1000));
  }
}
