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
public class AutonElevator extends CommandBase {
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private double mEndTime;
  private boolean isRising;
  private static final Timer m_Timer = new Timer();

  public AutonElevator(double time, boolean isRising ) {
   // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Elevator);
    mEndTime = time;
    this.isRising = isRising;
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
    if(isRising){
      RobotContainer.m_Elevator.descend();
    }

    else {
      RobotContainer.m_Elevator.ascend();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Elevator.condescend();
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
