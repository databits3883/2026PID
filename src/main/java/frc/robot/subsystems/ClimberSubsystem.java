// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSoftLimit;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase 
{
  private SparkMax m_primary_motor = new SparkMax(Constants.Climber.PRIMARY_MOTOR_ID, MotorType.kBrushless);
  private SparkLimitSwitch m_climberLimitSwitch_forward = null;
  private SparkLimitSwitch m_climberLimitSwitch_reverse = null;
  private SparkMax m_secondary_motor = new SparkMax(Constants.Climber.SECONDARY_MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder m_climberEncoder;
  private SparkSoftLimit m_climberClimbingLimit = null;

  private double m_climberPower = Constants.Climber.MAX_POWER;
  private boolean m_isClimberRunning = false;

  private double lastPositionRead = 0;
    
  /**
   * Initialize the motor and other components
   */
  public ClimberSubsystem() 
  {     
    //Create the limit switches
    m_climberLimitSwitch_forward = m_primary_motor.getForwardLimitSwitch();
    m_climberLimitSwitch_reverse = m_primary_motor.getReverseLimitSwitch();
    m_climberClimbingLimit = m_primary_motor.getReverseSoftLimit();
    //create the encoder from the motor
    m_climberEncoder = m_primary_motor.getEncoder();
    //Track the last time we read the encoder
    lastPositionRead = m_climberEncoder.getPosition();
  }

  //This will determine if the motor is stalled
  public boolean isStalled() 
  {
    //If the motor is supposed to be running, check the current draw
    double motorAppliedOutput = m_primary_motor.getAppliedOutput();
    double currentPosition = getCurrentClimberPosition();
    double deltaPosition = currentPosition - lastPositionRead;
    System.out.println("IsStalled-MotorAppliedOutput="+motorAppliedOutput+ ", deltaPosition="+deltaPosition);
    //TODO read the values above and see what values should be a stalled 

    return false;
  }
  /**
   * returns true if the intake 4 bar is at reverse limit
   * @return
   */
  public boolean isClimberReverseLimit()
  {
    return m_climberLimitSwitch_reverse.isPressed();
  }
  /**
   * returns true if the intake 4 bar is at forward limit
   * @return
   */
  public boolean isClimberForwardLimit()
  {
    return m_climberLimitSwitch_forward.isPressed();
  }

  public boolean isAtClimbLimit()
  {
    //TODO: This will probably be defined as a soft limit
    //verify with build team
    return m_climberClimbingLimit.isReached();    
  }

  /**
   * Returns true if the intake motor is running
   * @return
   */
  public boolean isClimberRunning()
  {
    return m_isClimberRunning;
  }

  /**
   * Stop the intake from spinning
   */
  public void stopClimber()
  {
    //turn off motor
    m_secondary_motor.setVoltage(0);
    m_isClimberRunning = false;
  }

  /**
   * Run the intake motor to the default configured velocity
   */
  public void runClimber()
  {
    //Run to the default target power
    runClimber(m_climberPower);
  }
  /** Run the motor to a given power */
  public void runClimber(double targetVoltage)
  {
    m_secondary_motor.setVoltage(targetVoltage);
    if (targetVoltage != 0) m_isClimberRunning = true; 
    else m_isClimberRunning = false;
  }
  /**
   * Run the intake reverse speed
   */
  public void reverseClimber()
  {
    runClimber(-1*m_climberPower);
  }
  public void reverseClimber(double reversePowerLevel)
  {
    runClimber(reversePowerLevel);
  }

  /**
   * Returns the current position from the primary encoder
   * @return
   */
  public double getCurrentClimberPosition()
  { 
    return (m_climberEncoder.getPosition());  
  }

  @Override
  public void periodic() 
  {
    //Update the last position reading
    lastPositionRead = getCurrentClimberPosition();
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}
