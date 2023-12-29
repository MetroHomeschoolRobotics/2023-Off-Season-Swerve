package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class Teleop extends CommandBase {
  private CommandXboxController xboxController;
  
  private Drivetrain drivetrain;
  
  public Teleop(CommandXboxController xboxController_, Drivetrain drivetrain_) {
    addRequirements(drivetrain_);
    xboxController = xboxController_;
    drivetrain = drivetrain_;
  }
  
  public void initialize() {}
  
  public void execute() {
    drivetrain.translateSpin(xboxController.getLeftX(), -xboxController.getLeftY(), xboxController.getRightX());
  }
  
  public void end(boolean interrupted) {}
  
  public boolean isFinished() {
    return false;
  }
}
