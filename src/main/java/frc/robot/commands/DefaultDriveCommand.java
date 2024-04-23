// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultDriveCommand extends Command {
  
  private SwerveSubsystem mSwerve;
  /** Creates a new DefaultDrive. */
  public DefaultDriveCommand(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerve = swerve;
    addRequirements(mSwerve);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = -Input.getJoystickX();
    double y = -Input.getJoystickY();
    double rot = -Input.getRot();

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
