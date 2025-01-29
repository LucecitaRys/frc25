package frc.robot.subsystems;

import com.revrobotics.spark.config.LimitSwitchConfig.Type;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.LimelightHelpers;
import frc.robot.Constants;



public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem mShooterSubsystem;
    private final TalonFX motSup;
    private final TalonFX motInf;

    private final SparkMax motorP;
    private SparkLimitSwitch m_forwardLimit;
  private SparkLimitSwitch m_reverseLimit;
private SparkClosedLoopController pidController;
   private SparkAbsoluteEncoder absoluteEncoder;
   private SparkMaxConfig config = new SparkMaxConfig();

    public ShooterSubsystem() {
      
motSup = new TalonFX(Constants.MotorConstants.id_ms);
motInf = new TalonFX(Constants.MotorConstants.id_mi);
motorP = new SparkMax(Constants.MotorConstants.id_mp, MotorType.kBrushless);

SparkClosedLoopController pidController = motorP.getClosedLoopController();
m_forwardLimit = motorP.getForwardLimitSwitch();
m_reverseLimit = motorP.getReverseLimitSwitch();
absoluteEncoder = motorP.getAbsoluteEncoder();


SparkMaxConfig config = new SparkMaxConfig();
config
.inverted(false);

config.closedLoop
    .pid(1.0, 0.0, 0.0)
    .outputRange(-1, 1);

config.limitSwitch
    .forwardLimitSwitchType(Type.kNormallyClosed)
    .forwardLimitSwitchEnabled(true)
    .reverseLimitSwitchType(Type.kNormallyOpen)
    .reverseLimitSwitchEnabled(true);

   


double tx = LimelightHelpers.getTX("x");
double x = 9.7- tx; 
double speedS =  4000; //(rpm)
double speedI =(1/x)+4000; 


    }

    public void setShooterSpeed(double speedSup, double speedInf) {
      
  

        motSup.set(speedSup);
        motInf.set(speedInf);
       

       //  SmartDashboard.putNumber("Shooter Speed Superior", speedSup* 600 / 2048);
       // SmartDashboard.putNumber("Shooter Speed Inferior", speedInf* 600 / 2048);
    }
    public void setDutyCycleOutput(double speed) {

      motorP.set(speed *0.5);
     // pidController.setReference(speed, ControlType.kDutyCycle);
     // SmartDashboard.putNumber("speed", speed);

  }

    public static ShooterSubsystem getInstance (){
      if (mShooterSubsystem== null){
        mShooterSubsystem = new ShooterSubsystem();
      }
      return mShooterSubsystem;
    }
  }