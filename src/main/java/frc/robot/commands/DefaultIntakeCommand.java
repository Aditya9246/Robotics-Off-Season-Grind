// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends Command {
  private IntakeSubsystem mIntake;
  /** Creates a new DefaultIntakeCommand. */
  public DefaultIntakeCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntake.stopIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Input.getLeftTrigger() > 0.1){
      if(mIntake.pieceIntaken()){
        mIntake.setIntakeSpeed(IntakeConstants.HANDOFF_SPEED);
      }
      else{
        mIntake.setIntakeSpeed(Input.getLeftTrigger());
      }
    }
    else{
      mIntake.stopIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
