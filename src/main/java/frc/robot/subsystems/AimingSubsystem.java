// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AimingSubsystem extends SubsystemBase {
  /** Creates a new AimingSubsystem. */
  private CANSparkMax mLeftWristMotor = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax mRightWristMotor = new CANSparkMax(2, MotorType.kBrushless);
  private DutyCycleEncoder mThroughBoreEncoder = new DutyCycleEncoder(3);

  private double mCurrentWristRotDeg = 0;
  private double mDesiredWristRotDeg = 0;

  private PIDController mWristController = AimingConstants.mWristPIDConstants.toWPIController();


  public AimingSubsystem() {
    config();
  }

  public void config(){
    mLeftWristMotor.restoreFactoryDefaults();
    mRightWristMotor.restoreFactoryDefaults();

    mLeftWristMotor.setInverted(!AimingConstants.WRIST_LEFT_IS_NORMAL);
    mWristController.setTolerance(AimingConstants.WRIST_TOLERANCE_DEG);

    mRightWristMotor.follow(mLeftWristMotor, true);

    mLeftWristMotor.burnFlash();
    mRightWristMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}