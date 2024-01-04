// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElbowSubsystem extends SubsystemBase {

  private CANSparkMax elbowDrive;
  private CANSparkMax elbowDriveTwo;

  private SparkMaxPIDController elbow_pidController;
  private RelativeEncoder elbow_encoder;
  // PID variables used to be public in sample code
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double currentPosition;
  // private SparkMaxAbsoluteEncoder elbow_AbsoluteEncoder;

  /** Creates a new ElbowSubsystem. */
  public ElbowSubsystem() {
    elbowDrive = new CANSparkMax(Constants.elbowDrive_ID, MotorType.kBrushless);
    elbowDriveTwo = new CANSparkMax(Constants.elbowDriveTwo_ID, MotorType.kBrushless);

    elbowDrive.restoreFactoryDefaults();
    elbowDriveTwo.restoreFactoryDefaults();

    elbow_pidController = elbowDrive.getPIDController();

    elbowDriveTwo.follow(elbowDrive);

    elbow_encoder = elbowDrive.getEncoder();

    // elbow_AbsoluteEncoder = elbowDrive.getAbsoluteEncoder(Type.kDutyCycle);

    // PID Coefficients
    kP = 2;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // Set PID coefficients
    elbow_pidController.setP(kP);
    elbow_pidController.setI(kI);
    elbow_pidController.setD(kD);
    elbow_pidController.setIZone(kIz);
    elbow_pidController.setFF(kFF);
    elbow_pidController.setOutputRange(kMinOutput, kMaxOutput);

    elbowDrive.setIdleMode(IdleMode.kBrake);

    elbowDrive.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    elbowDrive.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);

    elbowDrive.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.elbowSoftForward);
    elbowDrive.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.elbowSoftReverse);
  }

  public void ElbowUp() {
    elbowDrive.set(-0.5);
    //elbowDriveTwo.set(-0.5);
  }

  public void ElbowDown() {
    elbowDrive.set(0.5);
    //elbowDriveTwo.set(0.5);
  }

  public void ElbowOff() {
    elbowDrive.set(0);
    //elbowDriveTwo.set(0);
  }

  public void ElbowHalfLowerAuto() {
    elbow_pidController.setReference(
        Constants.ElbowHalfLowerAutoValue, CANSparkMax.ControlType.kPosition);
    // elbow_pidController.setReference(Constants.ElbowHalfLowerAutoValue,
    // CANSparkMax.ControlType.kDutyCycle);
  }

  public void ElbowHomeAuto() {
    elbow_pidController.setReference(0, CANSparkMax.ControlType.kPosition);
    // elbow_pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
  }

  public void ElbowHighAuto() {
    elbow_pidController.setReference(Constants.elbowHighAuto, CANSparkMax.ControlType.kPosition);
    // elbow_pidController.setReference(Constants.elbowHighAuto,
    // CANSparkMax.ControlType.kDutyCycle);
  }

  public void ElbowMediumAuto() {
    elbow_pidController.setReference(Constants.elbowMediumAuto, CANSparkMax.ControlType.kPosition);
    // elbow_pidController.setReference(Constants.elbowMediumAuto,
    // CANSparkMax.ControlType.kDutyCycle);
  }

  public void ElbowGroundAuto() {
    elbow_pidController.setReference(Constants.elbowGroundAuto, CANSparkMax.ControlType.kPosition);
    // elbow_pidController.setReference(Constants.elbowGroundAuto,
    // CANSparkMax.ControlType.kDutyCycle);
  }

  public void GetEncodervalue() {
    currentPosition = elbow_encoder.getPosition();
  }

  public void ElbowEncoderReset() {
    elbow_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
