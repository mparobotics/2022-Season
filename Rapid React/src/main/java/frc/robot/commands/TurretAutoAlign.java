// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import java.lang.Math; // to use the Pi value in line 40

public class TurretAutoAlign extends CommandBase {
  /** Creates a new TurretAutoAlign. */


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);//offset on x axis
  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);//degrees, offset from target on y axis
  double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);//is target in view?
  double h1 = ShooterConstants.limelight_height; //corresponds to the case study graph in limelight document
  double h2 = ShooterConstants.hub_height; //corresponds to the case study graph in limelight document
  double a1 = ShooterConstants.limelight_angle; //degrees, corresponds to the case study graph in limelight document; angle between the camera and the ground
  double d ; //corresponds to the case study graph in limelight document; horizontal distance between the camera and the target
  
  boolean tapeFound;
  SmartDashboard.putNumber("LimelightX", tx);
  SmartDashboard.putNumber("LimelightY", ty);
  SmartDashboard.putNumber("LimelightX", tv);
  if (tv != 1) {tapeFound = false;}
  else {tapeFound = true;}
  SmartDashboard.putBoolean("limelight vision", tapeFound);
  if (tapeFound == true){
    double angleRad = Math.PI*(ty+a1)/180; // converting ty+a1 from degrees to radians
    d = (h2-h1) / Math.tan(angleRad); // caculate the distance
  SmartDashboard.putNumber("DistanceFromHub", d); 

    //Robot.driveSubsystem.teleop(0, 0);
    double heading_error = tx; //needs inverting?
    double steering_adjust = 0.0f;
    if (tx > 1.0) //to test deadzone
    {
        //steering_adjust = ShooterConstants.Kp * heading_error + ShooterConstants.min_command; //Kp is a number that does something, and the min command is the minimum needed for it to react
        steering_adjust = .1; //to test speed and inversion
        Robot.turretSubsystem.turnTurret(steering_adjust);//toTest
    }
  
    else if (tx < 1.0) //to test deadzone
    {
        //steering_adjust = ShooterConstants.Kp * heading_error - ShooterConstants.min_command;
        steering_adjust = -.1; //to test speed and inversion
        Robot.turretSubsystem.turnTurret(steering_adjust);//toTest
    }

    else{new TurretNeutral();}


    
  }
  else{new TurretNeutral();}

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
