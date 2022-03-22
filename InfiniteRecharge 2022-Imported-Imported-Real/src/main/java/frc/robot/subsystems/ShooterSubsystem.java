/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TurretAutoAlign;
/**
 * this subsystem sets up and directly manipulates the high goal shooter
 */
public class ShooterSubsystem extends SubsystemBase {

  //declaring and intializing shooter motor
  private final static WPI_TalonFX falconShooter = new WPI_TalonFX(ShooterConstants.FALCON_shooter_ID); 
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
  PIDController pidController = new PIDController(ShooterConstants.kP, .001, 0);
  BangBangController BangBang = new BangBangController(); //instantiartes bangbang motor controller
  public double ShooterSpeed = falconShooter.getSelectedSensorVelocity();
  public static Timer timer;
  public Servo servo;
  private double integral, setpoint = 0;
  private double error;
  //static CANSparkMax hoodMotor = new CANSparkMax(ShooterConstants.hood_motor_ID, MotorType.kBrushless);
  //public static RelativeEncoder m_encoder = hoodMotor.getEncoder();


  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    servo = new Servo(ShooterConstants.LIMELIGHT_SERVO_ID);
    setServo(ShooterConstants.LIMELIGHT_ANGLE_SETPOINT);
    falconShooter.setNeutralMode(NeutralMode.Coast);
    //hoodMotor.setIdleMode(IdleMode.kBrake);
    falconShooter.setInverted(false); //invert motor
    falconShooter.configOpenloopRamp(2); // 0.5 seconds from neutral to full output (during open-loop control)
    falconShooter.configClosedloopRamp(2); // 0 disables ramping (during closed-loop control)
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //SmartDashboard.putNumber("Hood Angle", m_encoder.getPosition());

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

   public void ShootBangBang(double yspeed) {
    //setpoint = getSetpoint();

    double m_setpoint;
    if (yspeed == 2) {
      new TurretAutoAlign(new TurretSubsystem());
       m_setpoint = getV() * 12;
    }
   

    else {m_setpoint = (((yspeed / .00009) + 400));
      if (m_setpoint == 400){
        falconShooter.set(.05);

      }
     }
    boolean canIShoot;
    //alignHood();
    
    
    if (m_setpoint != 400){
      
    falconShooter.setVoltage(BangBang.calculate(falconShooter.getSelectedSensorVelocity(), 
                                                (m_setpoint)) * 12 + .000148 * feedforward.calculate(m_setpoint)
                            );
    //.000148, .0005
    if (Math.abs(falconShooter.getSelectedSensorVelocity() - m_setpoint) < 2000){
      
      canIShoot = true;
   
    }
    else{canIShoot = false;
    
    }
  
    SmartDashboard.putBoolean("Shooter Reved Up", canIShoot);}
    //falconShooter.set(BangBang.calculate(falconShooter.getSelectedSensorVelocity(), (setpoint)));
   //bang bang based on size of d from the hub

  }


  public void ShootLow() {
    //setpoint = getSetpoint();
    
    boolean canIShoot;
    //alignHood();
    double m_setpoint = 4500;
    falconShooter.setVoltage(BangBang.calculate(falconShooter.getSelectedSensorVelocity(), (m_setpoint)) * 12 + .0005 * feedforward.calculate(m_setpoint));
    //.000148
    if (Math.abs(falconShooter.getSelectedSensorVelocity() - m_setpoint) < 900){
      
      canIShoot = true;
      
    }
    else{canIShoot = false;
    
    
    }
    SmartDashboard.putBoolean("Shooter Reved Up", canIShoot);
    //falconShooter.set(BangBang.calculate(falconShooter.getSelectedSensorVelocity(), (setpoint)));
   //bang bang based on size of d from the hub

  }

  public void ShootReverse() {
    //setpoint = getSetpoint();
    
    boolean canIShoot;
    //alignHood();
    double m_setpoint = -4500;
    falconShooter.set(-.3);
    //.000148
   
    //falconShooter.set(BangBang.calculate(falconShooter.getSelectedSensorVelocity(), (setpoint)));
   //bang bang based on size of d from the hub

  }

  public void ShootPid(){
    setpoint = 8000;
    boolean canIShoot;
    
    falconShooter.setVoltage(pidController.calculate(falconShooter.getSelectedSensorVelocity(), setpoint) );

  }

  public double getSetpoint () {
    double speedToGet = getV();
    setpoint = speedToGet; //needs math here
    return setpoint;
  }

  public static double sensorVelocityToRPM(double sensorUnitsPer100MS) {
    double sensorUnitsPerSecond = sensorUnitsPer100MS * 10.0;
    double revolutionsPerSecond = sensorUnitsPerSecond / 2048.0;
    double revolutionsPerMinute = revolutionsPerSecond * 60.0;
    return revolutionsPerMinute;
  }

  public double getV(){
    double d = TurretSubsystem.getDistance();
    double speedToGet;
    /*if (d < .5) {speedToGet = 7500;}
    else if (d < 1) {speedToGet = 7500;}
    else if (d < 1.5) {speedToGet = 7500;}
    else if (d < 2) {speedToGet = 8000;}
    else if (d < 2.5) {speedToGet = 8500;}
    else if (d < 3) {speedToGet = 9000;}
    else if (d < 3.5) {speedToGet = 9500;}
    else if (d < 4) {speedToGet = 10000;}
    else if (d < 4.5) {speedToGet = 11000;}
    else if (d < 5) {speedToGet = 12500;}
    else if (d < 5.5) {speedToGet = 13000;}
    else if (d < 6) {speedToGet = 12500;}
    else if (d < 6.5) {speedToGet = 14500;}
    */
    if (d < 7)
    {
      speedToGet = 1076 * d + 5970; 
    }
    else {speedToGet = 4500;}
    ///math

    SmartDashboard.putNumber("Flywheel Speed Needed", setpoint);
    return speedToGet;
  }

  public void alignHood() {
    double angle = 0;
    //double currentEncoder = m_encoder.getPosition();
    double encoderCountRequired = angle * ShooterConstants.hood_encoder_ratio;
    SmartDashboard.putNumber("Hood Encoder Needed", encoderCountRequired);
    /*if (currentEncoder < (encoderCountRequired - ShooterConstants.hood_min_command)) {
      adjustHood(-.25); //to test speed and inversion
    }

    else if (currentEncoder > (encoderCountRequired + ShooterConstants.hood_min_command)) {
      adjustHood(.25); //to test speed and inversion
    }

    else {
      adjustHood(0);
    }*/

  }

  public static void adjustHood(double speed) {
    //hoodMotor.set(speed);
  }

  public double getAngle(){
    double d = TurretSubsystem.getDistance();
    double v = getV(); 
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

   public void encoderReset() {
    falconShooter.setSelectedSensorPosition(0);
    //m_encoder.setPosition(0);
  }


   public void shootPIControl() {
     falconShooter.set(((shooterPI() + setpoint) / ShooterConstants.SHOOTER_MAX_VELOCITY));
     SmartDashboard.putNumber("Shooter PI", shooterPI()+ setpoint);
     SmartDashboard.putNumber("Shooter Power", (shooterPI() + setpoint) / ShooterConstants.SHOOTER_MAX_VELOCITY);
   }


}
