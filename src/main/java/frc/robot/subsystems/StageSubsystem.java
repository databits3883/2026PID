// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StageSubsystem extends SubsystemBase {
   // PID Gains and Motion Profile Constraints
    private static double kP = Constants.StageConstants.KP;
    private static double kI = Constants.StageConstants.KI;
    private static double kD = Constants.StageConstants.KD;
    private static double maxOutput = Constants.StageConstants.MAX_OUTPUT;
    private static final double kMaxVelocity = Constants.StageConstants.MAX_VELOCITY;
    private static final double kMaxAcceleration = Constants.StageConstants.MAX_ACCELERATION;
    
    private SparkMax m_motor = new SparkMax(Constants.StageConstants.STAGE_MOTOR_ID, MotorType.kBrushless);
    
       
    //private SparkMaxConfig m_config = new SparkMaxConfig();
    private SparkMaxConfig m_baseConfig = new SparkMaxConfig();
    private SparkClosedLoopController closedLoopController = m_motor.getClosedLoopController();
    private RelativeEncoder stageEncoder;


  public StageSubsystem() {
   
      stageEncoder = m_motor.getEncoder();
      
        
        m_baseConfig.closedLoop
                        .p(kP)
                        .i(kI)
                        .d(kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange((-1 * maxOutput),maxOutput)
                        .feedForward.kV(12.0/5767); // set PID and 1/3 max speeds                        
        m_baseConfig.idleMode(IdleMode.kCoast);
       

        //Update the motoro config to use PID
        m_motor.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.setDefaultNumber("Stage Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Stage Run Moter", false);
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(SmartDashboard.getBoolean("Stage Run Moter", false)) {
      double targetVelocity = SmartDashboard.getNumber("Stage Target Velocity", 0);
      closedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
