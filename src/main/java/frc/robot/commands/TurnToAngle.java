// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.subsystems.WheelDrive;

// /** A command that will turn the robot to the specified angle. */
// public class TurnToAngle extends PIDCommand {
//   /**
//    * Turns to robot to the specified angle.
//    *
//    * @param targetAngleDegrees The angle to turn to
//    * @param drive The drive subsystem to use
//    */
//   public TurnToAngle(double targetAngleDegrees, WheelDrive drive) {
//     super(
//         new PIDController(0.001,0.00003,0),
//         // Close loop on heading
//         drive::getSensorValue,
//         // Set reference to target
//         targetAngleDegrees,
//         // Pipe output to turn robot
//         output -> drive.driveAngleMotor(output),
//         // Require the drive
//         drive);

//     // Set the controller to be continuous (because it is an angle controller)
//     getController().enableContinuousInput(-180, 180);
//     // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
//     // setpoint before it is considered as having reached the reference
//     getController()
//         .setTolerance(5, 10);
//   }

//   @Override
//   public boolean isFinished() {
//     // End when the controller is at the reference.
//     return getController().atSetpoint();
//   }
// }
