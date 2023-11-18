package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
// import frc.robot.commands.TurnToAngle;

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

public SwerveDrive (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
    this.backRight = backRight;
    this.backLeft = backLeft;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
}
  public void drive (double x1, double y1, double x2, boolean slowMode2) {
    // double r = Math.sqrt ((L * L) + (W * W));
    boolean isForward = true;
    y1*=-1;
    
    double dx1 = deadzone(x1,0.2);
    double dx2 = deadzone(x2,0.2)*Constants.scalarSwerve;
    double dy1 = deadzone(y1,0.2);
    double a = dx1 - dx2 * (L / Constants.OperatorConstants.drive_r);
    double b = dx1 + dx2 * (L / Constants.OperatorConstants.drive_r);
    double c = dy1 - dx2 * (W / Constants.OperatorConstants.drive_r);
    double d = dy1 + dx2 * (W / Constants.OperatorConstants.drive_r);
    
    //double magnitude = deadzone(Math.hypot(x1, y1)*0.9, 0.05);
    //double angle = Math.atan2(x1,y1);
    double backRightSpeed = Math.sqrt((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt((b * b) + (c * c));


    backRightAngle = (Math.atan2 (a, c) / Math.PI) * 180;
    backLeftAngle = (Math.atan2 (a, d) / Math.PI) * 180;
    frontRightAngle = (Math.atan2 (b, c) / Math.PI) * 180;
    frontLeftAngle = (Math.atan2 (b, d) / Math.PI) * 180;



    // backRight.driveSpeedMotor(backRightSpeed*0.3);
    // new TurnToAngle(backRightAngle, backRight).schedule();

    // backLeft.driveSpeedMotor(backLeftSpeed*0.3);
    // new TurnToAngle(backLeftAngle, backLeft).schedule();;

    // frontRight.driveSpeedMotor(frontRightSpeed*0.3);
    // new TurnToAngle(frontRightSpeed, frontRight).schedule();;

    // frontLeft.driveSpeedMotor(frontLeftSpeed*0.3);
    // new TurnToAngle(frontLeftSpeed, frontLeft).schedule();

    if(slowMode2){
    optimizeDrive(backRight, backRightSpeed*0.2,backRightAngle);
    optimizeDrive (backLeft, backLeftSpeed*0.2, backLeftAngle);
    optimizeDrive (frontRight, frontRightSpeed*0.2, frontRightAngle);
    optimizeDrive (frontLeft,frontLeftSpeed*0.2, frontLeftAngle);
    }
    else{
    optimizeDrive(backRight, backRightSpeed,backRightAngle);
    optimizeDrive (backLeft, backLeftSpeed, backLeftAngle);
    optimizeDrive (frontRight, frontRightSpeed, frontRightAngle);
    optimizeDrive (frontLeft,frontLeftSpeed, frontLeftAngle);
    }
  }
  /** Creates a new ExampleSubsystem. */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void AutonDrive(double direction, double speed){

    optimizeDrive(backRight, speed,direction);
    optimizeDrive (backLeft,speed, direction);
    optimizeDrive (frontRight, speed, direction);
    optimizeDrive (frontLeft,speed, direction);
  }
  public void stopAll(){
    backRight.stopMotor();
    backLeft.stopMotor();
    frontRight.stopMotor();;
    frontLeft.stopMotor();;
  }
  private double deadzone(double value, double deadzone)
  {
      if (Math.abs(value) < deadzone)
      // if (Math.abs(Math.sqrt(x * x + y * y)) < deadzone)

      {
          return 0;
      }
      return value;
  }

  // private double speedScaling(double x, double y, double deadzone){
  //   if (Math.abs(Math.sqrt(x * x + y * y)) < deadzone)
  //     {
  //         return 0;
  //     }

  //     return value;

  // }
  public void zeroizeEncoders(){
      backRight.moveAngleMotor(backRight.getSensorValue(),0.0,true);
      backLeft.moveAngleMotor(backLeft.getSensorValue(),0.0,true);
      frontLeft.moveAngleMotor(frontLeft.getSensorValue(),0.0,false);
      frontRight.moveAngleMotor(frontRight.getSensorValue(),0.0,true);

  }


  public void optimizeDrive(WheelDrive unit,double speed,double controllerAngle){
    double motorAngle = unit.getSensorValue();
    // double motorAngle2;
    boolean isForward = true;
//     if (motorAngle > 0){
//        motorAngle2 = motorAngle - 180;
//      } 

//     else {
//        motorAngle2 = motorAngle + 180;
//     }

// if (Math.abs(motorAngle2 - controllerAngle) > 180){
//   distanceM2 = 360 - Math.abs(motorAngle2 - controllerAngle);
// }

// else {
//   distanceM2 = Math.abs(motorAngle2 - controllerAngle);
// }

// if (Math.abs(motorAngle - controllerAngle) > 180){
//   distanceM1 = 360 - Math.abs(motorAngle2 - controllerAngle);
// }

// else {
//   distanceM1 = Math.abs(motorAngle - controllerAngle);
// }

// if (distanceM2 <  distanceM1){
//   isForward = false;
//   unit.drive(speed,motorAngle2,controllerAngle,isForward);
// }
// else {
//   unit.drive(speed,motorAngle,controllerAngle,isForward);
// }
  unit.drive(speed,motorAngle,controllerAngle,isForward);
  }
  }

