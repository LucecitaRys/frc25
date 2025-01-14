package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.LimelightHelpers;
import frc.robot.Constants;



public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem mShooterSubsystem;
    private final TalonFX motSup;
    private final TalonFX motInf;
 

    public ShooterSubsystem() {
      
        motSup = new TalonFX(Constants.MotorConstants.id_ms);
  motInf = new TalonFX(Constants.MotorConstants.id_mi);
  

double tx = LimelightHelpers.getTX("x");
double x = 9.7- tx; 
double speedS =  4000; //(rpm)
double speedI =(1/x)+4000; 

    }

    public void setShooterSpeed(double speedSup, double speedInf) {
      
  

        motSup.set(speedSup);
        motInf.set(speedInf);
       

         SmartDashboard.putNumber("Shooter Speed Superior", speedSup* 600 / 2048);
        SmartDashboard.putNumber("Shooter Speed Inferior", speedInf* 600 / 2048);
    }

    public static ShooterSubsystem getInstance (){
      if (mShooterSubsystem== null){
        mShooterSubsystem = new ShooterSubsystem();
      }
      return mShooterSubsystem;
    }
  }