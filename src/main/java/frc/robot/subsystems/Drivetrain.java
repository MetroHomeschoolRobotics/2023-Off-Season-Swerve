package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveConstants;

public class Drivetrain extends SubsystemBase {


  private SwerveModule frontRightMod = new SwerveModule(
      "Front Right", 
      0, 0, 
      1, 
      swerveConstants.swerveModuleFR.angleMotorReversed, swerveConstants.swerveModuleFR.driveMotorReversed);
  
  private SwerveModule frontLeftMod = new SwerveModule(
      "Front Left", 
      1, 2, 
      3, 
      swerveConstants.swerveModuleFL.angleMotorReversed, swerveConstants.swerveModuleFL.driveMotorReversed);
  
  private SwerveModule backRightMod = new SwerveModule(
      "Back Right", 
      2, 4, 
      5, 
      swerveConstants.swerveModuleBR.angleMotorReversed, swerveConstants.swerveModuleBR.driveMotorReversed);
  
  private SwerveModule backLeftMod = new SwerveModule(
      "Back Left", 
      3, 6, 
      7, 
      swerveConstants.swerveModuleBL.angleMotorReversed, swerveConstants.swerveModuleBL.driveMotorReversed);
  



  public void periodic() {}
  



  public void translate(double inputX, double inputY) {
    double angle = Math.atan(inputY / inputX) * 180/Math.PI;

    frontRightMod.setAngle(angle);
    frontLeftMod.setAngle(angle);
    backRightMod.setAngle(angle);
    backLeftMod.setAngle(angle);

    double input = Math.sqrt(Math.pow(inputX, 2) + Math.pow(inputY, 2));

    frontRightMod.setSpeed(input);
    frontLeftMod.setSpeed(input);
    backRightMod.setSpeed(input);
    backLeftMod.setSpeed(input);
  }
  
  public void spin(double inputX) {
    double angle = 45;
    
    frontRightMod.setAngle(angle);
    frontLeftMod.setAngle(angle);
    backRightMod.setAngle(angle);
    backLeftMod.setAngle(angle);
    
    frontRightMod.setSpeed(inputX);
    frontLeftMod.setSpeed(inputX);
    backRightMod.setSpeed(inputX);
    backLeftMod.setSpeed(inputX);
  }
  
  public void translateSpin(double speedX, double speedY, double turnX) {
    double TurnVectorXM1 = turnX * Math.cos(Math.PI/4);
    double TurnVectorYM1 = turnX * Math.sin(Math.PI/4);
    double TurnVectorXM2 = turnX * Math.cos(Math.PI/4 + Math.PI/2);
    double TurnVectorYM2 = turnX * Math.sin(Math.PI/4 + Math.PI/2);
    double TurnVectorXM3 = turnX * Math.cos(Math.PI/4 + Math.PI);
    double TurnVectorYM3 = turnX * Math.sin(Math.PI/4 + Math.PI);
    double TurnVectorXM4 = turnX * Math.cos(Math.PI/4 + (Math.PI*3)/2);
    double TurnVectorYM4 = turnX * Math.sin(Math.PI/4 + (Math.PI*3)/2);

    double M1VectorAddedX = TurnVectorXM1 + speedX;
    double M1VectorAddedY = TurnVectorYM1 + speedY;
    double M2VectorAddedX = TurnVectorXM2 + speedX;
    double M2VectorAddedY = TurnVectorYM2 + speedY;
    double M3VectorAddedX = TurnVectorXM3 + speedX;
    double M3VectorAddedY = TurnVectorYM3 + speedY;
    double M4VectorAddedX = TurnVectorXM4 + speedX;
    double M4VectorAddedY = TurnVectorYM4 + speedY;

    double M1VectorAngle = Math.atan2(M1VectorAddedX, M1VectorAddedY) * 180/Math.PI;
    double M2VectorAngle = Math.atan2(M2VectorAddedX, M2VectorAddedY) * 180/Math.PI;
    double M3VectorAngle = Math.atan2(M3VectorAddedX, M3VectorAddedY) * 180/Math.PI;
    double M4VectorAngle = Math.atan2(M4VectorAddedX, M4VectorAddedY) * 180/Math.PI;

    double M1VectorLength = Math.sqrt(Math.pow(M1VectorAddedX, 2) + Math.pow(M1VectorAddedY, 2));
    double M2VectorLength = Math.sqrt(Math.pow(M2VectorAddedX, 2) + Math.pow(M2VectorAddedY, 2));
    double M3VectorLength = Math.sqrt(Math.pow(M3VectorAddedX, 2) + Math.pow(M3VectorAddedY, 2));
    double M4VectorLength = Math.sqrt(Math.pow(M4VectorAddedX, 2) + Math.pow(M4VectorAddedY, 2));

    double vectorLengthMax1 = Math.max(Math.abs(M1VectorLength), Math.abs(M2VectorLength));
    double vectorLengthMax2 = Math.max(Math.abs(M3VectorLength), Math.abs(M4VectorLength));
    double vectorLengthMaxT = Math.max(vectorLengthMax1, vectorLengthMax2);

    double M1VectorLengthNorm = 0;
    double M2VectorLengthNorm = 0;
    double M3VectorLengthNorm = 0;
    double M4VectorLengthNorm = 0;

    SmartDashboard.putNumber("Vector Max Length", vectorLengthMaxT);

    if (vectorLengthMaxT >= 1) {
      M1VectorLengthNorm = M1VectorLength / vectorLengthMaxT;
      M2VectorLengthNorm = M2VectorLength / vectorLengthMaxT;
      M3VectorLengthNorm = M3VectorLength / vectorLengthMaxT;
      M4VectorLengthNorm = M4VectorLength / vectorLengthMaxT;
    } else {
      M1VectorLengthNorm = M1VectorLength;
      M2VectorLengthNorm = M2VectorLength;
      M3VectorLengthNorm = M3VectorLength;
      M4VectorLengthNorm = M4VectorLength;
    } 

    frontRightMod.setAngle(M1VectorAngle);
    frontLeftMod.setAngle(M2VectorAngle);
    backRightMod.setAngle(M3VectorAngle);
    backLeftMod.setAngle(M4VectorAngle);

    frontRightMod.setSpeed(M1VectorLengthNorm);
    frontLeftMod.setSpeed(M2VectorLengthNorm);
    backRightMod.setSpeed(M3VectorLengthNorm);
    backLeftMod.setSpeed(M4VectorLengthNorm);
  }
}
