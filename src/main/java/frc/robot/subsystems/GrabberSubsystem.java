// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {
WPI_TalonSRX grabber = new WPI_TalonSRX(Constants.grabber_ID);
  //private CANSparkMax grabber;

  /** Creates a new ElbowSubsystem. */
  public GrabberSubsystem() {
    

    grabber.configFactoryDefault();
  }

  public void grabberOff() {
    grabber.set(0);
  }

  public void grabberOut() {
    grabber.set(-0.4);
  }

  public void grabberIn() {
    grabber.set(0.4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
