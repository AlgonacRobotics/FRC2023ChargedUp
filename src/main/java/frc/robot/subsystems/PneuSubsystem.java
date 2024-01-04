// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneuSubsystem extends SubsystemBase {
  /** Creates a new PneuSubsystem. */

  // Define Pneumatic Solenoids
  public DoubleSolenoid collector_GrabRelease =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, Constants.pneuCollectorA, Constants.pneuCollectorB);

  public PneuSubsystem() {}

  public void CollectorOpen() {
    collector_GrabRelease.set(Value.kReverse);
  }

  public void CollectorClosed() {
    collector_GrabRelease.set(Value.kForward);
  }

  public void CollectorOff() {
    collector_GrabRelease.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
