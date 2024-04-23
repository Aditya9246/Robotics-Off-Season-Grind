// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

  // Pigeon - for pose, gyro stuff
  private static Pigeon2 mPigeon2;

  // PID Controllers
  private PIDController chassisRotPID =  new PIDController(0, 0, 0);
  private PIDController chassisXPID = new PIDController(0, 0, 0);
  private PIDController chassisYPID = new PIDController(0, 0, 0);

  // Rotation Motors
  private CANSparkMax Rot_FLMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax Rot_FRMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax Rot_BLMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax Rot_BRMotor = new CANSparkMax(0, MotorType.kBrushless);

  // Translation Motors
  private CANSparkMax Trans_FLMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax Trans_FRMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax Trans_BLMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax Trans_BRMotor = new CANSparkMax(0, MotorType.kBrushless);


  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    configureDevices();
  }

  public void configureDevices() {

    Rot_FLMotor.restoreFactoryDefaults();
    Rot_FRMotor.restoreFactoryDefaults();
    Rot_BLMotor.restoreFactoryDefaults();
    Rot_FRMotor.restoreFactoryDefaults();
    
    Trans_FLMotor.restoreFactoryDefaults();
    Trans_FRMotor.restoreFactoryDefaults();
    Trans_BLMotor.restoreFactoryDefaults();
    Trans_BRMotor.restoreFactoryDefaults();

    // set inverted

    // set tolerances

    // mLeftElevatorEncoder.setPositionConversionFactor(AimingConstants.ELEVATOR_ROTATIONS_TO_METERS);
    // mRightElevatorEncoder.setPositionConversionFactor(AimingConstants.ELEVATOR_ROTATIONS_TO_METERS);

    Rot_FLMotor.burnFlash();
    Rot_FRMotor.burnFlash();
    Rot_BLMotor.burnFlash();
    Rot_BRMotor.burnFlash();

    Trans_FLMotor.burnFlash();
    Trans_FRMotor.burnFlash();
    Trans_BLMotor.burnFlash();
    Trans_BRMotor.burnFlash();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
