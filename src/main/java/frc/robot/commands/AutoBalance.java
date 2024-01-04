// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalance extends CommandBase {
  // assumes already on charging station.
  private Drivetrain drive;
  private PIDController balancePid = new PIDController(0.005, 0, 0);

  double endTime = 0;
  double duration = 0;

  /** Creates a new AutoBalance. */
  public AutoBalance(Drivetrain swerve) {
    this.drive = swerve;
    addRequirements(swerve);
    SmartDashboard.putData("AutoBalancePID", balancePid);
    // Use addRequirements() here to declare subsystem dependencies.

    // duration = ms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balancePid.reset();
    balancePid.setTolerance(1.5);

    endTime = System.currentTimeMillis() + duration;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = drive.getPitch();

    var vx = balancePid.calculate(pitch, 1.11);

    drive.drive(vx, 0, 0, false);

    SmartDashboard.putNumber("autobalance/roll", pitch);
    SmartDashboard.putNumber("autobalance/vx", vx);
    // SmartDashboard.putNumber("auto")
  }

  // Called once the commak nd ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.enableXstance();
    drive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return System.currentTimeMillis() >= endTime;
  }
}
