// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class BallShooter extends CommandBase {
  /** Creates a new spinWheel. */

  public double ticks2RPM = 4096/10/60;
  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;

  private PIDController pid = new PIDController(0, 0, 0);

  private WPI_TalonSRX flyWheel = new WPI_TalonSRX(0);
  private WPI_TalonSRX feedWheel = new WPI_TalonSRX(1);

  public BallShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    flyWheel.configFactoryDefault();
    feedWheel.configFactoryDefault();

    flyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

  }

  public double getRPM(){
    return flyWheel.getSelectedSensorVelocity()/ticks2RPM;
  }

  
  //im not sure if this will work but i think it might if you call setspeed later
  public void setSpeed(double setPoint){
    flyWheel.set(ControlMode.PercentOutput, pid.calculate(getRPM(), setPoint));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
