// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheel_Velocity;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;


public class FlyWheelVelocityRun extends CommandBase {
  /** Creates a new FlyWheelVelocityRun. */
  
  private final FlyWheel_Velocity m_flywWheel_Velocity;
  public FlyWheelVelocityRun(FlyWheel_Velocity f) {
  
     m_flywWheel_Velocity = f;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setLedMode(LightMode.eOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double d = TurretSubsystem.getDistance();
    double speedToGet;

    //d = SmartDashboard.getNumber("distance sim", 3);
    SmartDashboard.putBoolean("Lined Up To Shoot", false);
    if (d < .5) {speedToGet = 4000;}
    else if (d < 1) {speedToGet = 4000;}
    else if (d < 1.5) {speedToGet = 4000;}
    else if (d < 2) {speedToGet = 7050;}
    //else if (d < 2.35) {speedToGet = 7800; SmartDashboard.putBoolean("Lined Up To Shoot", false);} //to test on practice field
    else if (d < 2.5) {speedToGet = 7550; SmartDashboard.putBoolean("Lined Up To Shoot", true);}
    //else if (d < 2.75) {speedToGet = 9000; SmartDashboard.putBoolean("Lined Up To Shoot", false);} //to test on practice field
    else if (d < 3) {speedToGet = 8075;}
    else if (d < 3.5) {speedToGet = 8300;} //9200
    else if (d < 4) {speedToGet = 9250;}
    else if (d < 4.5) {speedToGet = 9400;}
    else if (d < 5) {speedToGet = 9600;}
    else if (d < 5.5) {speedToGet = 10000;}
    else if (d < 6) {speedToGet = 10650;}
    else if (d < 6.5) {speedToGet = 11000;}
    else {speedToGet = 4000;} 
    
   /*d = Math.round(d * 10) / 10;
    speedToGet = (1404 * d) + 5232; //4560 1620
    if (d > 6.5) {speedToGet = 4000;}*/

    m_flywWheel_Velocity.my_Flywheel_Velocity(speedToGet);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywWheel_Velocity.myFlyWheel_PercentOut(0);
    Limelight.setLedMode(LightMode.eOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
