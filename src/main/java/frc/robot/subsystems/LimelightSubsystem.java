package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

 public class LimelightSubsystem extends SubsystemBase{

   public static boolean aiming = false;
   

  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        // final double STEER_K = 0.03;                    // how hard to turn toward the target
        // final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        // final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        // final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        // double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        // double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        // double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (ta > 0.5)
        {
         aiming = true;
        }
        else{
            aiming = false;
        }

       

        // // Start with proportional steering
        // double steer_cmd = tx * STEER_K;
        // // m_LimelightSteerCommand = steer_cmd;

        // // try to drive forward until the target area reaches our desired area
        // double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // // don't let the robot drive too fast into the goal
        // if (drive_cmd > MAX_DRIVE)
        // {
        //   drive_cmd = MAX_DRIVE;
        // }
        // m_LimelightDriveCommand = drive_cmd;
  }

  public double getLimelightY(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getLimelightX(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public boolean isLimelightXRange(){
    double x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    return (x > -20 && x < 20);
  }
}