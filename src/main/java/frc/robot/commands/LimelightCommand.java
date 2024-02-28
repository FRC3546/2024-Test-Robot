package frc.robot.commands;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCommand extends Command {
  private final LimelightSubsystem limelightSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param LimelightSubsystem The subsystem used by this command.
   */
  public LimelightCommand(LimelightSubsystem limelightSubsystem) {
    
   this.limelightSubsystem = limelightSubsystem;
    addRequirements(limelightSubsystem);
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    limelightSubsystem.Update_Limelight_Tracking();

  }
  
//   LimelightSubsystem.Update_Limelight_Tracking();


  @Override 
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
