// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LaunchSubsystem extends SubsystemBase {
   // PID Gains and Motion Profile Constraints
    private static double kP = Constants.LaunchConstants.KP;
    private static double kI = Constants.LaunchConstants.KI;
    private static double kD = Constants.LaunchConstants.KD;
    private static double maxOutput = Constants.LaunchConstants.MAX_OUTPUT;
    //private static final double kMaxVelocity = Constants.LaunchConstants.MAX_VELOCITY;
    //private static final double kMaxAcceleration = Constants.LaunchConstants.MAX_ACCELERATION;
    
    private SparkMax m_motor_a = new SparkMax(Constants.LaunchConstants.LAUNCH_MOTOR_ID_A, MotorType.kBrushless);
    private SparkMax m_motor_b = new SparkMax(Constants.LaunchConstants.LAUNCH_MOTOR_ID_B, MotorType.kBrushless);
       
    //private SparkMaxConfig m_config = new SparkMaxConfig();
    private SparkMaxConfig m_baseConfig = new SparkMaxConfig();
    private SparkClosedLoopController closedLoopController_a = m_motor_a.getClosedLoopController();
    //private SparkClosedLoopController closedLoopController_b = m_motor_b.getClosedLoopController();
    private RelativeEncoder launchEncoder_a=null;
    //private RelativeEncoder launchEncoder_b=null;
    
    //Initializers for Shuffleboard objects
    private Double targetVelocity = Double.valueOf(0);
    private Boolean runMotor = Boolean.valueOf(false);
    private Boolean updatePID = Boolean.valueOf(false);
    private Double pGain = Double.valueOf(kP);
    private Double iGain = Double.valueOf(kI);
    private Double dGain = Double.valueOf(kD);
    private Double iAccum = Double.valueOf(0);
    private Double currentVelocity = Double.valueOf(0);
    

  public LaunchSubsystem() {
   
      launchEncoder_a = m_motor_a.getEncoder();      
        
      m_baseConfig.closedLoop
                        .p(kP)
                        .i(kI)
                        .d(kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange((-1 * maxOutput),maxOutput)
                        //.feedForward.kV(12.0/917) // set PID 
                        ;                        
        m_baseConfig.idleMode(IdleMode.kCoast);
        //m_baseConfig.encoder.velocityConversionFactor(0.5);

        //Update the motoro config to use PID
        m_motor_a.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //update config for follower
        m_baseConfig.follow(Constants.LaunchConstants.LAUNCH_MOTOR_ID_A,true);
        m_motor_b.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  

        SmartDashboard.setDefaultNumber("Launch Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Launch Run Motor", false);
        SmartDashboard.setDefaultBoolean("Launch Update PID", false);
        SmartDashboard.putNumber("Launch P Gain", kP);
        SmartDashboard.putNumber("Launch I Gain", kI);
        SmartDashboard.putNumber("Launch D Gain", kD);
        SmartDashboard.putNumber("Launch IAccum", 0);
        SmartDashboard.putNumber("Launch Current Velocity", 0);

        

        Shuffleboard.getTab("Launch Testing").addNumber("Launch Target Velocity", ()->getTargetVelocity());
        Shuffleboard.getTab("Launch Testing").addBoolean("Launch Run Motor", ()->getRunMotor());
        Shuffleboard.getTab("Launch Testing").addBoolean("Launch Update PID", ()->getUpdatePID());
        Shuffleboard.getTab("Launch Testing").addNumber("Launch P Gain", ()->getPGain());
        Shuffleboard.getTab("Launch Testing").addNumber("Launch I Gain", ()->getIGain());
        Shuffleboard.getTab("Launch Testing").addNumber("Launch D Gain", ()->getDGain());
        Shuffleboard.getTab("Launch Testing").addNumber("Launch IAccum", ()->getIAccum());
        Shuffleboard.getTab("Launch Testing").addNumber("Launch Current Velocity", ()->getCurrentVelocity());

  }

  //Getters and Setters for Shuffleboard objects
  private void setTargetVelocity(Double targetSpeed){
    targetVelocity = targetSpeed;
  }
  private Double getTargetVelocity(){
    return targetVelocity;
  }

  private void setRunMotor(Boolean runningMotor){
    runMotor = runningMotor;
  }
  private Boolean getRunMotor(){
    return runMotor;
  }

  private void setUpdatePID(Boolean isUpdatePID){
    updatePID = isUpdatePID;
  }
  private Boolean getUpdatePID(){
    return updatePID;
  }
  
  private void setPGain(Double pSet){
    pGain = pSet;
  }
  private Double getPGain(){
    return pGain;
  }

  private void setIGain(Double iSet){
    iGain = iSet;
  }
  private Double getIGain(){
    return iGain;
  }

  private void setDGain(Double dSet){
    dGain = dSet;
  }
  private Double getDGain(){
    return dGain;
  }

  private void setIAccum(Double IA){
    iAccum = IA;
  }
  private Double getIAccum(){
    return iAccum;
  }

  private void setCurrentVelocity(Double currentSpeed){
    currentVelocity = currentSpeed;
  }
  private Double getCurrentVelocity(){
    return currentVelocity;
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
 public void stop()
    {
        //set the current
        closedLoopController_a.setSetpoint(0, ControlType.kVelocity);
        //turn off motor
        m_motor_a.setVoltage(0);
        //turn off the control mode
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(SmartDashboard.getBoolean("Launch Run Motor", false)) {
      double targetVelocity = SmartDashboard.getNumber("Launch Target Velocity", 0);
      closedLoopController_a.setSetpoint(targetVelocity, ControlType.kVelocity);
    } else {
      stop();
    }


    if(SmartDashboard.getBoolean("Launch Update PID", false)){
      SmartDashboard.putBoolean("Launch Update PID", false);
        
        
            //Read PID values
            // read PID coefficients from SmartDashboard
            double p = SmartDashboard.getNumber("Launch P Gain", 0);
            double i = SmartDashboard.getNumber("Launch I Gain", 0);
            double d = SmartDashboard.getNumber("Launch D Gain", 0);
            
            // if PID coefficients on SmartDashboard have changed, write new values to controller
            boolean updatePID = false;
            if((p != kP)) { updatePID = true;  kP = p; }
            if((i != kI)) { updatePID = true;  kI = i; }
            if((d != kD)) { updatePID = true; kD = d; }

            if (updatePID)
            {
                //Update the PID on close loopController
                m_baseConfig.closedLoop
                        .p(kP)
                        .i(kI)
                        .d(kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange((-1 * maxOutput),maxOutput); // set PID and 1/3 max speeds
                //Update the motoro config to use PID
                m_motor_a.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            } //end if updatePID
    } 
    
        SmartDashboard.putNumber("Launch Current Velocity", launchEncoder_a.getVelocity());
        SmartDashboard.putNumber("Launch IAccum", closedLoopController_a.getIAccum());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
