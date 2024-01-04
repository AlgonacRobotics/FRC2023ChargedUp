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

public class ElevatorSubsystem extends SubsystemBase {

  private CANSparkMax elevatorOne;
  private CANSparkMax elevatorTwo;

  private SparkMaxPIDController elevator_pidController;
  private RelativeEncoder elevator_encoder;
  // PID variables used to be public in sample code
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double currentElevatorPosition;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorOne = new CANSparkMax(Constants.elevatorOne_ID, MotorType.kBrushless);
    elevatorTwo = new CANSparkMax(Constants.elevatorTwo_ID, MotorType.kBrushless);

    elevatorOne.restoreFactoryDefaults();
    elevatorTwo.restoreFactoryDefaults();

    // elevatorTwo.setInverted(true);
    // elevatorOne.setInverted(false);

    elevator_pidController = elevatorOne.getPIDController();

    elevator_encoder = elevatorOne.getEncoder();

    // PID Coefficients
    kP = 2;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // Set PID coefficients
    elevator_pidController.setP(kP);
    elevator_pidController.setI(kI);
    elevator_pidController.setD(kD);
    elevator_pidController.setIZone(kIz);
    elevator_pidController.setFF(kFF);
    elevator_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // elevatorTwo.follow(elevatorOne);

    elevatorTwo.follow(elevatorOne, true);

    elevatorOne.setIdleMode(IdleMode.kBrake);
    elevatorTwo.setIdleMode(IdleMode.kBrake);

    elevatorOne.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    elevatorOne.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);

    elevatorOne.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward, Constants.elevatorSoftForward);
    elevatorOne.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse, Constants.elevatorSoftReverse);
  }

  public void ElevatorUp() {
    elevatorOne.set(0.25);
  }

  public void ElevatorDown() {
    elevatorOne.set(-0.25);
  }

  public void ElevatorOff() {
    elevatorOne.set(0);
  }

  public void ElevatorHomeAuto() {
    elevator_pidController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public void ElevatorHighAuto() {
    elevator_pidController.setReference(
        Constants.elevatorHighAuto, CANSparkMax.ControlType.kPosition);
  }

  public void ElevatorMediumAuto() {
    elevator_pidController.setReference(
        Constants.elevatorMediumAuto, CANSparkMax.ControlType.kPosition);
  }

  public void ElevatorLowAuto() {
    elevator_pidController.setReference(
        Constants.elevatorLowAuto, CANSparkMax.ControlType.kPosition);
  }

  public void GetElevatorEncodervalue() {
    currentElevatorPosition = elevator_encoder.getPosition();
  }

  public void ElevatorEncoderReset() {
    elevator_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
