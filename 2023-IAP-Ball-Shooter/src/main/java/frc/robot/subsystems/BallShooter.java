// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class BallShooter extends SubsystemBase {
  /** Creates a new spinWheel. */

  public double ticks2RPS = 4096/10;
  private double setPoint;

  private PIDController pid = new PIDController(Constants.FlywheelPIDConsts.pidP, Constants.FlywheelPIDConsts.pidI, Constants.FlywheelPIDConsts.pidD);
  private SimpleMotorFeedforward feedF = new SimpleMotorFeedforward(Constants.FeedForwardConst.kS, Constants.FeedForwardConst.kV, Constants.FeedForwardConst.kA);

  private WPI_TalonSRX flyWheel = new WPI_TalonSRX(4);
  private WPI_TalonSRX feedWheel = new WPI_TalonSRX(3);

  private boolean onOrOff = false;


  public BallShooter() {
    flyWheel.configFactoryDefault();
    feedWheel.configFactoryDefault();

    flyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    resetEncoder();

    flyWheel.setNeutralMode(NeutralMode.Coast);

    pid.setTolerance(0.05);

  }

  public double getRPS(){
    return flyWheel.getSelectedSensorVelocity()/ticks2RPS*-1;
  }
//maube delete this
  public double getVelocity(){
    return getRPS()*Constants.flyCircumference/100.0;
  }
  
  public void setFeed(double speed){
    feedWheel.set(ControlMode.PercentOutput, speed);
  }
  //combine PID with feedforward
  public void setSpeed(double setPoint){
    if(setPoint == 0){
      flyWheel.set(ControlMode.PercentOutput, 0);
    } else{
      flyWheel.setVoltage(pid.calculate(getRPS(), setPoint) + feedF.calculate(setPoint));
    }
  }

  public void resetEncoder(){
    flyWheel.setSelectedSensorPosition(0);
  }

  public void setSetpoint(double setPoint){
    pid.setSetpoint(setPoint);
  }

  public boolean atSetpoint(){
    return pid.atSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void periodic() {
    if(RobotContainer.getJoy().getRawButtonPressed(1)){
      onOrOff = !onOrOff;
      if(onOrOff){
        setFeed(0.9);
      } else {
        setFeed(0);
      }
    }
    if(RobotContainer.getJoy().getRawButtonPressed(3)){
      setPoint = 97;
    }
    if(RobotContainer.getJoy().getRawButtonPressed(5)){
      setPoint = 85;
    }
    if(RobotContainer.getJoy().getRawButtonPressed(4)){
      setPoint = 90;
    }
    if(RobotContainer.getJoy().getRawButtonPressed(2)){
      setPoint = 0;
    }

    setSetpoint(setPoint);
    setSpeed(setPoint);
    
    SmartDashboard.putNumber("RPS", getRPS());
    SmartDashboard.putBoolean("setpoint", pid.atSetpoint());
    SmartDashboard.putNumber("bang", pid.calculate(getRPS(), setPoint));
    SmartDashboard.putNumber("Feed Forward", 0.9 * feedF.calculate(setPoint)/12.0);
    }
}
