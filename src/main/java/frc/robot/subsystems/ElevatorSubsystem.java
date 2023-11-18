package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


public class ElevatorSubsystem extends SubsystemBase {
private Talon elevator_motor1;
private DigitalInput limitSwitch = new DigitalInput(0);
private DigitalInput limitSwitch2 = new DigitalInput(1);

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem(int channel)
  {      elevator_motor1 = new Talon(channel);


  }

  public void ascend(){
      if(!limitSwitch.get()){
        elevator_motor1.set(0);
      }
      else{
        elevator_motor1.setInverted(false);
        elevator_motor1.set(.40);
      }
        
  }
  //This goes up
  public void descend(){
    if(!limitSwitch2.get()){
      elevator_motor1.set(0);
    }
    else{
        elevator_motor1.setInverted(true);
        elevator_motor1.set(.9);
    }
       
  }

  public void condescend(){
      elevator_motor1.stopMotor();

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}