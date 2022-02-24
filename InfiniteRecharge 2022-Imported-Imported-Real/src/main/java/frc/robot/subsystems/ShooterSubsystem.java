/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
/**
 * this subsystem sets up and directly manipulates the high goal shooter
 */
public class ShooterSubsystem extends SubsystemBase {

  //declaring and intializing shooter motor
  private final static WPI_TalonFX falconShooter = new WPI_TalonFX(ShooterConstants.FALCON_shooter_ID); 
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
  BangBangController BangBang = new BangBangController(); //instantiartes bangbang motor controller
  public double ShooterSpeed = falconShooter.getSelectedSensorVelocity();
  public static Timer timer;
  public Servo servo;
  private double integral, setpoint = 0;
  private double error;
  static WPI_TalonSRX hoodMotor = new WPI_TalonSRX(ShooterConstants.hood_motor_ID);


  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    servo = new Servo(ShooterConstants.LIMELIGHT_SERVO_ID);
    setServo(ShooterConstants.LIMELIGHT_ANGLE_SETPOINT);
    falconShooter.setNeutralMode(NeutralMode.Coast);
    hoodMotor.setNeutralMode(NeutralMode.Brake);
    falconShooter.setInverted(false); //invert motor
    SmartDashboard.putNumber("Hood Encoder Count", hoodMotor.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterSpeed", falconShooter.getSelectedSensorVelocity());
    RobotContainer.helms.setRumble(RumbleType.kLeftRumble, Math.log10((falconShooter.getSelectedSensorVelocity() ) / 100) );
    RobotContainer.helms.setRumble(RumbleType.kRightRumble, Math.log10((falconShooter.getSelectedSensorVelocity() ) / 100));
  }

  public void setServo(double degrees) {
    servo.setAngle(degrees);
  }

  public void setSetpoint(double set) {
    setpoint = set;
  }

  /**
   * sets the speed of the shooter to a constant speed
   */
  public void shootBall() {
    falconShooter.set(ShooterConstants.HIGH_GOAL_SPEED);

   
   }

   public void ShootBangBang() {
    //setpoint = getSetpoint();
    setpoint = 10000;
    falconShooter.set(BangBang.calculate(falconShooter.getSelectedSensorVelocity(), (setpoint) + 0.5 * feedforward.calculate(setpoint)));
   //bang bang based on size of d from the hub

  }

  public double getSetpoint () {
    double speedToGet = getV();
    setpoint = speedToGet; //needs math here
    return setpoint;
  }

  public double getV(){
    double d = TurretSubsystem.getDistance();
    double speedToGet;
    if (d < .5) {speedToGet = 7.101;}
    else if (d < 1) {speedToGet = 6.614;}
    else if (d < 1.5) {speedToGet = 6.797;}
    else if (d < 2) {speedToGet = 6.797;}
    else if (d < 2.5) {speedToGet = 7.132;}
    else if (d < 3) {speedToGet = 7.407;}
    else if (d < 3.5) {speedToGet = 7.711;}
    else if (d < 4) {speedToGet = 8.138;}
    else if (d < 4.5) {speedToGet = 8.443;}
    else if (d < 5) {speedToGet = 8.63;}
    else if (d < 5.5) {speedToGet = 8.9;}
    else if (d < 6) {speedToGet = 9.14;}
    else if (d < 6.5) {speedToGet = 9.33;}
    else if (d < 7) {speedToGet = 9.45;}
    else if (d < 7.5) {speedToGet = 9.81;}
    else if (d < 8) {speedToGet = 10.06;}
    else if (d < 8.5) {speedToGet = 10.24;}
    else if (d < 9) {speedToGet = 10.49;}
    else {speedToGet = 8.5;}
    ///math
    SmartDashboard.putNumber("Flywheel Speed Needed", setpoint);
    return speedToGet;
  }

  public void alignHood() {
    double angle = getAngle();
    double currentEncoder = hoodMotor.getSelectedSensorPosition();
    double encoderCountRequired = angle * ShooterConstants.hood_encoder_ratio;
    SmartDashboard.putNumber("Hood Encoder Needed", encoderCountRequired);
    if (currentEncoder < (encoderCountRequired - ShooterConstants.min_command)) {
      adjustHood(.5); //to test speed and inversion
    }

    else if (currentEncoder > (encoderCountRequired + ShooterConstants.min_command)) {
      adjustHood(-.5); //to test speed and inversion
    }

    else {
      adjustHood(0);
    }

  }

  public void adjustHood(double speed) {
    hoodMotor.set(speed);
  }

  public double getAngle(){
    double d = TurretSubsystem.getDistance();
    double v = getV(); //m/s
    double beta = Math.atan(Math.abs((ShooterConstants.hub_height-ShooterConstants.limelight_height)/(d)));
    double dist = Math.sqrt(Math.pow(ShooterConstants.hub_height-ShooterConstants.limelight_height, 2) + Math.pow(d, 2));
    double angleTopPart = dist * 9.81 * Math.pow(Math.cos(beta), 2);
    double angleBottomPart = Math.pow(v, 2) + Math.sin(beta) + (beta / 2);
    double angle = .5 * Math.asin(angleTopPart / angleBottomPart);
    return angle;}




   public void shootBallAuto() {
    falconShooter.set(.45);
   
   } 

   public void reverseShooter() {
     falconShooter.set(ShooterConstants.HIGH_GOAL_SPEED);
   }

   /**
    * stops the shooter motor
    */
   public static void stopShooter() {
     falconShooter.set(0);
   }

   private double shooterPI() {
     error = setpoint - falconShooter.getSelectedSensorVelocity();
     //integral += error*.02;
     return ShooterConstants.SHOOTER_P*error;
   }

   public void shootPIControl() {
     falconShooter.set(((shooterPI() + setpoint) / ShooterConstants.SHOOTER_MAX_VELOCITY));
     SmartDashboard.putNumber("Shooter PI", shooterPI()+ setpoint);
     SmartDashboard.putNumber("Shooter Power", (shooterPI() + setpoint) / ShooterConstants.SHOOTER_MAX_VELOCITY);
   }


}
