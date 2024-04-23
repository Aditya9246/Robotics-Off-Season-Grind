// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Input;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax mIntakeMotorInner = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax mIntakeMotorOuter = new CANSparkMax(2, MotorType.kBrushless);
  private static final DigitalInput mBeamBreak = new DigitalInput(3);
  
  public IntakeSubsystem() {
    config();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void config(){
    mIntakeMotorInner.restoreFactoryDefaults();
    mIntakeMotorOuter.restoreFactoryDefaults();

    mIntakeMotorInner.setInverted(IntakeConstants.INTAKE_INVERTED_INNER);
    mIntakeMotorInner.enableVoltageCompensation(IntakeConstants.NOMINAL_VOLTAGE);

    mIntakeMotorOuter.setInverted(IntakeConstants.INTAKE_INVERTED_OUTER);
    mIntakeMotorOuter.enableVoltageCompensation(IntakeConstants.NOMINAL_VOLTAGE);

    mIntakeMotorInner.setIdleMode(IdleMode.kBrake);
    mIntakeMotorOuter.setIdleMode(IdleMode.kBrake);

    mIntakeMotorInner.burnFlash();
    mIntakeMotorOuter.burnFlash();
  }

  // -1.0 to 1.0
  public void setIntakeSpeed(double speed){
    mIntakeMotorInner.set(speed);
    mIntakeMotorOuter.set(speed);
  }

  public void runIntake(){
    mIntakeMotorInner.set(IntakeConstants.ACTIVE_INTAKE_SPEED);
    mIntakeMotorOuter.set(IntakeConstants.ACTIVE_INTAKE_SPEED);
  }

  public void stopIntake(){
    mIntakeMotorInner.set(0.0);
    mIntakeMotorOuter.set(0.0);
  }

  //TODO: check what the sensor returns in both conditions
  public boolean pieceIntaken(){
    return mBeamBreak.get();
  }
}
