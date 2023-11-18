// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JointCommand extends CommandBase {
  private final ElevatorSubsystem m_subsystemA;
  private final ArmSubsystem m_subsystemB;
  private int state;

  /** Creates a new JointCommand. */
  public JointCommand(ElevatorSubsystem subsystem1, ArmSubsystem subsystem2, int state) {

    this.state = state;
    m_subsystemA = subsystem1;
    m_subsystemB = subsystem2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem1, subsystem2);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state == 1){
      m_subsystemA.ascend();
      m_subsystemB.ascend();
    }

    else {
      m_subsystemA.condescend();
      m_subsystemB.condescend();
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
