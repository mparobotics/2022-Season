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
  TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final FlyWheel_Velocity m_flywWheel_Velocity;
  public FlyWheelVelocityRun(FlyWheel_Velocity f) {
  
     m_flywWheel_Velocity = f; //use more than just f

     //THERE SHOULD BE AN ADDED REQUIREMENT HERE. DONT SKIP THIS STEP

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setLedMode(LightMode.eOn); //turns on the evil light of doom
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double d = m_turretSubsystem.getDistance(); //gets the distance from the hub with the limelight
    //while in BallShoot, i noted why subsystem interaction can be problematic, an exception* is when you are simply using
    //a class that pulls a number. I would still avoid doing this
    
    double t = m_turretSubsystem.getTv(); //checks to see if the target in the limelights vision
    double speedToGet; //our speed setpoint
    boolean correctColor; //we did not ever use the color sorter


    //d = SmartDashboard.getNumber("distance sim", 3);
    //.putBoolean("Lined Up To Shoot", false);
   /*if (d < .5) {speedToGet = 4000;}
    else if (d < 1) {speedToGet = 4000;}
    else if (d < 1.5) {speedToGet = 4000;}
    else if (d < 2) {speedToGet = 7700;}
    else if (d < 2.25) {speedToGet = 7380;} //to test on practice field
    else if (d < 2.5) {speedToGet = 7950;}
    else if (d < 2.75) {speedToGet = 8000;} //to test on practice field
    else if (d < 3) {speedToGet = 8275;}
    else if (d < 3.25) {speedToGet = 8325;}
    else if (d < 3.5) {speedToGet = 8400;} //9200
    else if (d < 4) {speedToGet = 7250;}
    else if (d < 4.5) {speedToGet = 7400;}
    else if (d < 5) {speedToGet = 7600;}
    else if (d < 5.5) {speedToGet = 10000;}
    else if (d < 6) {speedToGet = 10650;}
    else if (d < 6.5) {speedToGet = 11000;}
    else {speedToGet = 4000;} */
    
   //d = Math.round(d * 100) / 100;
    //speedToGet = (1204 * d) + 4032; //4560 1620  1404 5232
    if (d < 1.25) {speedToGet = (1460 * d) + 2100;} //1264 3927}
    else if (d < 3.5) {speedToGet = (1275 * d) + 4614;}
    else if (d < 5) {speedToGet = (1325 * d) + 4434;}
    else if (d < 6.5) {speedToGet = (1360 * d) + 4534;}
    else {speedToGet = 4000;}

    /*This is our speed calculation code
     * its not great
     * but it was a smartish idea
     * so how did we calculate these functions you may ask
     * well, originally we took data from a bunch of points every half meter (see commented function above)
     * "but what if it isn't perfectly at those half meters" you, the oh so enlightened reader asks
     * introducing, linear extrapolation. There's a proper way to do this in desmos or something probably
     * but this is FRC. we don't do things properly. Well i mean it worked*, but instead i used logger pro
     * (Which by all means can be used for this but it prolly made things harder than they needed to be).
     * I inputed the speeds seen above at the distances seen above, and used the slope as m, and the y-intercept as c
     * in a standard y=mx+b arrangement (x is the distance from the hub).
     * It turns out, however, that one y=mx+b function is not enough when you don't have a hood. 
     * who would have thought that speed effects the angle something shoots at
     * oops
     * 
     * so i turned it into a peicewise function
     * the rest of the edits were a result of guess and check
     * 
     * painful guess and check, as often i didn't know which function
     * 
     * *worked is subjective. the problem wasn't the linear extrapolation, it was the lack of the hood and me still trying to 
     * "shoot from anywhere". I could have been spending time on better things.
     */






    //correctColor = m_flywWheel_Velocity.GetColor();
    //if (correctColor = false) {speedToGet = 4000;}
    if (t != 1) {
      speedToGet = 3900;
    }

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
