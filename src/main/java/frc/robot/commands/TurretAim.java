// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAim  extends Command {
  private double targetAngle = 0;
  private Pose2d targetTagPose = null;
  private SwerveSubsystem swerveSubsystem = RobotContainer.drivebase;
  private boolean isEnabled = false;
  private boolean inPlayerArea = false;

  //Set up the target Poses of places turrent should aim to
  //TODO: The top/bottom should move to center of the player sections
  private Pose2d redHubPose = new Pose2d(11.3118646, 4.3902376, new Rotation2d(0)); /* 3 */
  private Pose2d redBottomPose = new Pose2d(11.9528844, 0.6444996, new Rotation2d(0)); /* 7 */
  private Pose2d redTopPose = new Pose2d(11.9528844, 7.4247756, new Rotation2d(0)); /* 12 */
  private Pose2d blueBottomPose = new Pose2d(4.6630844, 0.6444996, new Rotation2d(0)); /* 17 */
  private Pose2d blueHubPose = new Pose2d( 5.229174199999999, 4.0346376, new Rotation2d(0)); /* 20 */
  private Pose2d blueTopPose = new Pose2d(4.6630844, 7.4247756, new Rotation2d(0)); /* 22 */
  

  //These are the X, and Y values for the midline and the scoring zones, stub in close values, get on init
  private double midFieldY = Units.inchesToMeters(158.84);
  private double redXPlayer = Units.inchesToMeters(651 -182.11 );
  private double blueXPlayer = Units.inchesToMeters(182.11);

  //Set up the field object to allow us to show the current target on the field
  private final Field2d m_field = new Field2d();

  /** Creates a new runSpinner. */
  public TurretAim () 
  {
    System.out.println("TurretAim:const:about to run constructor");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSubsystem);

    //Set up the field positions
    midFieldY = redHubPose.getY();
    redXPlayer = redTopPose.getX();
    blueXPlayer = blueTopPose.getX();

    //find target to aim based on the robot position
    targetTagPose = findTargetToAim(swerveSubsystem.getPose());
  }

  /*
   * This will check the alliance and position of the robot
   * to determine what april tag the turret should point to
   */
  private Pose2d findTargetToAim(Pose2d robotPose)
  {
    Pose2d targetPose = null;
    boolean isRedAlliance = Robot.isRedAlliance;
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();

    if (isRedAlliance)
    {
      inPlayerArea = false;
      if (robotX > redXPlayer)
      {
        inPlayerArea = true;
        targetPose = redHubPose;
        System.out.println("picked target redHub");
      }
      else if (robotY > midFieldY)
      {
        targetPose = redTopPose;
        System.out.println("picked target redTop");
      }
      else
      {
        targetPose = redBottomPose;
        System.out.println("picked target redBottom");
      }
    } //end red alliance
    else
    {
      if (robotX < blueXPlayer)
      {
        inPlayerArea = true;
        targetPose = blueHubPose;
        System.out.println("picked target blueHub");
      }
      else if (robotY > midFieldY)
      {
        targetPose = blueTopPose;
        System.out.println("picked target blueTop");
      }
      else
      {
        targetPose = blueBottomPose;
        System.out.println("picked target bluBottom");
      }
    }  //End blue alliance
    //If no target is found turn off aim
    if (targetPose == null) end(false);
    m_field.getObject("Target").setPose(targetPose);
    return targetPose;
  }

  private double getAngle(Pose2d robotPose, Pose2d targetPose)
  {
    //Get current robotRotation
    double robotHeadingDeg = swerveSubsystem.getHeading().getDegrees();
    //Need to know what to do with these angles, add heading?  Subtract heading?
    double targetX = targetPose.getX();
    double targetY = targetPose.getY();
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();

    double theta = Math.atan2(targetY-robotY, targetX-robotX);
    //We need to invert this angle from clockwise to counterclockwise if we are not in player area
    if (!inPlayerArea) theta = theta * -1;

    //Get the rotation from the center of the robot to the target if the robot was facing 0 (blue)
    double relativeRotationDeg = Units.radiansToDegrees(theta);
    StringBuffer output = new StringBuffer();
    output.append("[tx:").append(targetX).append(",")
          .append("ty:").append(targetY).append(",")
          .append("rx:").append(robotX).append(",")
          .append("ry:").append(robotY).append(",")
          .append("\ntargetDeg:").append(relativeRotationDeg).append(",");

    //Take robots position into account
    relativeRotationDeg -= robotHeadingDeg;
    output.append("targetDegRobot:").append(relativeRotationDeg).append(",");
    //Put back to -359 to 359 range
    relativeRotationDeg = relativeRotationDeg % 360;
    output.append("finalTargetDeg:").append(relativeRotationDeg).append(",");
    System.out.println(output.toString());

    //Right now set target to just the angle to tag
    return relativeRotationDeg;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putData("Field",m_field);
    if (isEnabled) 
    {
      isEnabled = false;
      //If running turn targetting off
      end(false);
    }
    else isEnabled = true;

    Pose2d currentRobotPose = swerveSubsystem.getPose();
    //Find new target based on robot positon
    targetTagPose = findTargetToAim(swerveSubsystem.getPose());    
    targetAngle = getAngle(currentRobotPose, targetTagPose);
    setTurrentAngle(targetAngle);
  }

  private void setTurrentAngle(double angleSetPoint)
  {
      System.out.println("TurretAim:setTurrentAngle:about to run turret to " + targetAngle);
      RobotContainer.turretSubsystem.setTurretSetPoint(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentRobotPose = swerveSubsystem.getPose();
    //Find new target based on robot positon
    targetTagPose = findTargetToAim(swerveSubsystem.getPose());    
    targetAngle = getAngle(currentRobotPose, targetTagPose);
    setTurrentAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isEnabled = false;
    if (interrupted)
    {
      System.out.println("RunSpinnner:end:interrupted was called!");
      //We do not need to do anything special if this gets interrupted early
    }
    System.out.println("TurretAim:end:about to stop motor");
    RobotContainer.turretSubsystem.stop();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    //never stop until toggled off
    /*
    double currentAngle = RobotContainer.turretSubsystem.getCurrentTurretAngle();
    double currentRots = Units.degreesToRotations(currentAngle);
    double targetRots = Units.degreesToRotations(targetAngle);
    double deadbandRots = Units.degreesToRotations(DEADBAND);
    //If we get inside the deadband stop
    if (Math.abs(currentRots - targetRots) <= deadbandRots)
    {
      RobotContainer.turretSubsystem.stop();
      return true;
    } 
    else
     */
      return false;
   } 
}