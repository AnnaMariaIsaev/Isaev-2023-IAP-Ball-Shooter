// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.FeedForwardConst;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class BallShooter extends SubsystemBase {
  /** Creates a new spinWheel. */

  public double ticks2RPM = 4096/10/60;
  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;

  private PIDController pid = new PIDController(Constants.FlywheelPIDConsts.pidP, Constants.FlywheelPIDConsts.pidI, Constants.FlywheelPIDConsts.pidD);
  private SimpleMotorFeedforward feedF = new SimpleMotorFeedforward(Constants.FeedForwardConst.kS, Constants.FeedForwardConst.kV, Constants.FeedForwardConst.kA);

  private WPI_TalonSRX flyWheel = new WPI_TalonSRX(0);
  private WPI_TalonSRX feedWheel = new WPI_TalonSRX(1);


  public BallShooter() {
    flyWheel.configFactoryDefault();
    feedWheel.configFactoryDefault();

    flyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    resetEncoder();
    flyWheel.setNeutralMode(NeutralMode.Coast);
    // this is in meters per second I think
    pid.setTolerance(0.05);

  }

  public double getRPM(){
    return flyWheel.getSelectedSensorVelocity()/ticks2RPM;
  }
  
  //combine PID with feedforward
  //what unit is the setpoint?
  public void setSpeed(double setPoint){
    flyWheel.setVoltage(pid.calculate(getRPM(), setPoint) + feedF.calculate(setPoint));
  }

  public void resetEncoder(){
    flyWheel.setSelectedSensorPosition(0);
  }

  public boolean atSetpoint(){
    return pid.atSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void periodic() {
    if(RobotContainer.getJoy().getRawButtonPressed(6)){
      setSpeed(0.1);
    }
    if(RobotContainer.getJoy().getRawButtonPressed(5)){
      setSpeed(0);
    }
    if(RobotContainer.getJoy().getRawButtonPressed(4)){
      setSpeed(1000);
    }

    SmartDashboard.putNumber("RPM", getRPM());
  }
}
