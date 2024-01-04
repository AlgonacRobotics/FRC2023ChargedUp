// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FanSubsystem extends SubsystemBase {

  private CANSparkMax fanDrive;

  /** Creates a new ElbowSubsystem. */
  public FanSubsystem() {
    fanDrive = new CANSparkMax(Constants.fanDrive_ID, MotorType.kBrushless);

    fanDrive.restoreFactoryDefaults();
  }

  public void FanOff() {
    fanDrive.set(0);
  }

  public void FanOn() {
    fanDrive.set(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
