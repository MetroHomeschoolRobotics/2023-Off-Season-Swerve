package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private CANCoder angleEncoder;

  private CANSparkMax angleMotor;
  
  private CANSparkMax driveMotor;
  
  private PIDController turnPID = new PIDController(0.005, 0, 0);
  
  private String placement;

  private double angleOffset;
  
  public SwerveModule(
    String modulePlacement, 
    int angleEncoderID, int angleMotorID, 
    int driveMotorID,
    boolean angleMotorReversed, boolean driveMotorReversed,
    double _angleOffset) {
    angleEncoder = new CANCoder(angleEncoderID);

    angleMotor = new CANSparkMax(angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

    driveMotor.setInverted(driveMotorReversed);
    angleMotor.setInverted(angleMotorReversed);

    driveMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setIdleMode(IdleMode.kBrake);

    turnPID.enableContinuousInput(-180, 180);

    placement = modulePlacement;

    angleOffset = _angleOffset;
  }
  
  public void periodic() {
    SmartDashboard.putNumber(placement + " Module Angle", moduleAngle());
  }
  
  // Angle Motor
  public double moduleAngle() {
    return angleEncoder.getAbsolutePosition()+angleOffset;
  }
  
  public void setAngle(double theta) {
    
    if(theta >= 0){
      SmartDashboard.putNumber(placement + " Angle", theta);
    }else{
      SmartDashboard.putNumber(placement + " Angle", theta+360);
    }

    double anglePID = MathUtil.clamp(turnPID.calculate(moduleAngle(), theta), -1, 1);

    SmartDashboard.putNumber(placement + " PID", anglePID);

    angleMotor.set(anglePID);
  }
  
  public Boolean angleTurnFinished() {
    return turnPID.atSetpoint();
  }
  

  // Drive Motor
  public double getDistance() {
    return driveMotor.getEncoder().getPosition();
  }
  
  public void setSpeed(double speed) {
    SmartDashboard.putNumber(placement + " Speed", speed);
    driveMotor.set(speed);
  }
  
  // this could be used for destinguishing between modules
  public String getPlacement() {
    return placement;
  }
}
