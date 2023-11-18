// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ElevatorSubsystem;


/** An example command that uses an example subsystem. */
public class AutonClaw extends CommandBase {
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private double mEndTime;
  private boolean isOpening;
  private static final Timer m_Timer = new Timer();

  public AutonClaw(double time, boolean isOpening ) {
   // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Claw, RobotContainer.m_Elevator);
    mEndTime = time;
    this.isOpening = isOpening;
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
    if (isOpening){
      RobotContainer.m_Claw.openClaw();
    }

    else {
      RobotContainer.m_Claw.closeClaw();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
