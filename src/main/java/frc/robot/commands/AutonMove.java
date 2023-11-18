// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/** An example command that uses an example subsystem. */
public class AutonMove extends CommandBase {

  private static Timer m_Timer = new Timer();
  private double mEndTime =0;
  private double mSpeed = 0;
  private double mDirection = 0;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonMove(double time, double speed, double direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_swerveDrive);
    mEndTime = time;
    mDirection =direction;
    mSpeed = speed;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_swerveDrive.zeroizeEncoders();
    Robot.m_swerveDrive.AutonDrive(mDirection, mSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_swerveDrive.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Timer.get()>mEndTime){
      return true;
    }
    return false;
  }
}
