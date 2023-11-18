package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
// import frc.robot.commands.TurnToAngle;
import frc.robot.commands.SwerveDriveInputScaler;

public class SwerveDrive extends SubsystemBase {
  public final double L = 10;
  public final double W = 10;
  public double distanceM2;
  public double distanceM1;
  private WheelDrive backRight;
  private WheelDrive backLeft;
  private WheelDrive frontRight;
  private WheelDrive frontLeft;
  public double backRightAngle = 0;
  public double backLeftAngle = 0;
  public double frontRightAngle = 0;
  public double frontLeftAngle = 0;
  public double backRightSpeed = 0;
  public double backLeftSpeed = 0;
  public double frontRightSpeed = 0;
  public double frontLeftSpeed = 0;
  private SwerveDriveInputScaler inputScaler;

  public SwerveDrive(WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
    this.backRight = backRight;
    this.backLeft = backLeft;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
    this.inputScaler = new SwerveDriveInputScaler();
  }

  public void drive(double strafeInput, double forwardInput, double omegaInput, boolean slowMode) {
    
    // Your robot's wheelbase dimensions (length L and width W)
    double L = 10; // Replace with actual length
    double W = 10; // Replace with actual width

    // TODO: Commenting out for now, may need to bring back later.
    //boolean isForward = true;
    //forwardInput *= -1;

    // TODO: Will need to tune this.
    double strafeInputDz = deadzone(strafeInput, 0.2);
    double omegaInputDz = deadzone(omegaInput, 0.2) * Constants.scalarSwerve;
    double forwardInputDz = deadzone(forwardInput, 0.2);

    // Scale the inputs
    double[] scaledInputs = inputScaler.scaleInputs(strafeInputDz, forwardInputDz, omegaInputDz);
    double scaledStrafe = scaledInputs[0];
    double scaledForward = scaledInputs[1];
    double scaledOmega = scaledInputs[2];

    // Calculate the omega components
    double omegaL2 = scaledOmega * (L / 2);
    double omegaW2 = scaledOmega * (W / 2);

    // Drive equation vectors
    double A = scaledStrafe - omegaL2;
    double B = scaledStrafe + omegaL2;
    double C = scaledForward - omegaW2;
    double D = scaledForward + omegaW2;    

    // Calculate wheel speeds
    double backRightSpeed = Math.sqrt((A * A) + (C * C));
    double backLeftSpeed = Math.sqrt((A * A) + (D * D));
    double frontRightSpeed = Math.sqrt((B * B) + (C * C));
    double frontLeftSpeed = Math.sqrt((B * B) + (D * D));

    // Normalize the wheel speeds
    double maxSpeed = Math.max(Math.max(Math.max(Math.abs(backRightSpeed), Math.abs(backLeftSpeed)),
                                        Math.abs(frontRightSpeed)), Math.abs(frontLeftSpeed));
    if (maxSpeed > 1) {
        backRightSpeed /= maxSpeed;
        backLeftSpeed /= maxSpeed;
        frontRightSpeed /= maxSpeed;
        frontLeftSpeed /= maxSpeed;
    }    

    // Calculate wheel angles
    double backRightAngle = Math.atan2(A, C) * 180 / Math.PI;
    double backLeftAngle = Math.atan2(A, D) * 180 / Math.PI;
    double frontRightAngle = Math.atan2(B, C) * 180 / Math.PI;
    double frontLeftAngle = Math.atan2(B, D) * 180 / Math.PI;

    if (slowMode) {
      optimizeDrive(backRight, backRightSpeed * 0.2, backRightAngle);
      optimizeDrive(backLeft, backLeftSpeed * 0.2, backLeftAngle);
      optimizeDrive(frontRight, frontRightSpeed * 0.2, frontRightAngle);
      optimizeDrive(frontLeft, frontLeftSpeed * 0.2, frontLeftAngle);
    } else {
      optimizeDrive(backRight, backRightSpeed, backRightAngle);
      optimizeDrive(backLeft, backLeftSpeed, backLeftAngle);
      optimizeDrive(frontRight, frontRightSpeed, frontRightAngle);
      optimizeDrive(frontLeft, frontLeftSpeed, frontLeftAngle);
    }
  }

  /** Creates a new ExampleSubsystem. */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void AutonDrive(double direction, double speed) {

    optimizeDrive(backRight, speed, direction);
    optimizeDrive(backLeft, speed, direction);
    optimizeDrive(frontRight, speed, direction);
    optimizeDrive(frontLeft, speed, direction);
  }

  public void stopAll() {
    backRight.stopMotor();
    backLeft.stopMotor();
    frontRight.stopMotor();
    ;
    frontLeft.stopMotor();
    ;
  }

  private double deadzone(double value, double deadzone) {
    if (Math.abs(value) < deadzone)
    // if (Math.abs(Math.sqrt(x * x + y * y)) < deadzone)

    {
      return 0;
    }
    return value;
  }

  // private double speedScaling(double x, double y, double deadzone){
  // if (Math.abs(Math.sqrt(x * x + y * y)) < deadzone)
  // {
  // return 0;
  // }

  // return value;

  // }
  public void zeroizeEncoders() {
    backRight.moveAngleMotor(backRight.getSensorValue(), 0.0, true);
    backLeft.moveAngleMotor(backLeft.getSensorValue(), 0.0, true);
    frontLeft.moveAngleMotor(frontLeft.getSensorValue(), 0.0, false);
    frontRight.moveAngleMotor(frontRight.getSensorValue(), 0.0, true);

  }

  public void optimizeDrive(WheelDrive unit, double speed, double controllerAngle) {
    double motorAngle = unit.getSensorValue();
    // double motorAngle2;
    boolean isForward = true;
    // if (motorAngle > 0){
    // motorAngle2 = motorAngle - 180;
    // }

    // else {
    // motorAngle2 = motorAngle + 180;
    // }

    // if (Math.abs(motorAngle2 - controllerAngle) > 180){
    // distanceM2 = 360 - Math.abs(motorAngle2 - controllerAngle);
    // }

    // else {
    // distanceM2 = Math.abs(motorAngle2 - controllerAngle);
    // }

    // if (Math.abs(motorAngle - controllerAngle) > 180){
    // distanceM1 = 360 - Math.abs(motorAngle2 - controllerAngle);
    // }

    // else {
    // distanceM1 = Math.abs(motorAngle - controllerAngle);
    // }

    // if (distanceM2 < distanceM1){
    // isForward = false;
    // unit.drive(speed,motorAngle2,controllerAngle,isForward);
    // }
    // else {
    // unit.drive(speed,motorAngle,controllerAngle,isForward);
    // }
    unit.drive(speed, motorAngle, controllerAngle, isForward);
  }
}
