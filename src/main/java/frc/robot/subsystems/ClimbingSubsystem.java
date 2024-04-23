// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase {
  private CANSparkMax mLeftElevatorMotor = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax mRightElevatorMotor = new CANSparkMax(1, MotorType.kBrushless);

  private PIDController mElevatorController = AimingConstants.mElevatorPIDConstants.toWPIController();  
  private RelativeEncoder mLeftElevatorEncoder;
  private RelativeEncoder mRightElevatorEncoder;

  private double mCurrentElevatorDistanceIn = 0.0;
  private double mDesiredElevatorDistanceIn = 0.0;

  /** Creates a new ClimbingSubsystem. */
  public ClimbingSubsystem() {
    config();
  }

  private void config(){
    mLeftElevatorMotor.restoreFactoryDefaults();
    mRightElevatorMotor.restoreFactoryDefaults();

    mLeftElevatorMotor.setInverted(!AimingConstants.LEFT_ELEVATOR_IS_NORMAL);

    mElevatorController.setTolerance(AimingConstants.ELEVATOR_TOLERANCE_IN);

    mRightElevatorMotor.follow(mLeftElevatorMotor);

    mLeftElevatorEncoder.setPositionConversionFactor(AimingConstants.ELEVATOR_ROTATIONS_TO_METERS);
    mRightElevatorEncoder.setPositionConversionFactor(AimingConstants.ELEVATOR_ROTATIONS_TO_METERS);

    mLeftElevatorEncoder.setPosition(0);
    mRightElevatorEncoder.setPosition(0);

    mLeftElevatorMotor.burnFlash();
    mRightElevatorMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mDesiredElevatorDistanceIn = MathUtil.clamp(mDesiredElevatorDistanceIn, AimingConstants.MIN_ELEVATOR_DIST_METERS, AimingConstants.MAX_ELEVATOR_DIST_METERS);
    mDesiredElevatorDistanceIn = MathUtil.clamp(mDesiredElevatorDistanceIn, AimingConstants.MIN_ELEVATOR_DIST_METERS, AimingConstants.MAX_ELEVATOR_DIST_METERS);
    mCurrentElevatorDistanceIn = getCurrentElevatorDistance();

    //Temp Regular PID
    double elevatorPIDCalculation = mElevatorController.calculate(mCurrentElevatorDistanceIn, mDesiredElevatorDistanceIn);
    elevatorPIDCalculation = MathUtil.clamp(elevatorPIDCalculation, -AimingConstants.MAX_ELEVATOR_PID_CONTRIBUTION, AimingConstants.MAX_ELEVATOR_PID_CONTRIBUTION);

    mLeftElevatorMotor.set(elevatorPIDCalculation + AimingConstants.ELEVATOR_FEEDFORWARD_CONSTANT);
    SmartDashboard.putNumber("ElevatorPIDOutput", elevatorPIDCalculation);
  }

  public double getCurrentElevatorDistance(){
    return mLeftElevatorEncoder.getPosition();
  }

  public double getDesiredElevatorDistance() {
    return mDesiredElevatorDistanceIn;
  }

  public void setDesiredElevatorDistance(double distance) {
    mDesiredElevatorDistanceIn = distance;
  }

  public void setDesiredSetpoint(AimState state) {
    mDesiredElevatorDistanceIn = state.elevatorDistIn;
  }

  public void changeDesiredElevatorPosition(double increment) {
    mDesiredElevatorDistanceIn += increment;
  }
}
