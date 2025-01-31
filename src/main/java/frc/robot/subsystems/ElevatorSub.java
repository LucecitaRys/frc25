package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSub extends SubsystemBase {
    private static ElevatorSub mElevatorSub;
    private final TalonFX ElevatorR;
    private final TalonFX ElevatorL;
  private TalonFXConfiguration motorConf = new TalonFXConfiguration();


    public ElevatorSub() {

ElevatorR= new TalonFX(Constants.MotorConstants.id_er);
ElevatorL = new TalonFX(Constants.MotorConstants.id_el);
ElevatorL.setControl(new Follower(Constants.MotorConstants.id_er, false));

  Slot0Configs slot0Configs = new Slot0Configs(); 
slot0Configs.kP = 0;
slot0Configs.kI = 0;
slot0Configs.kD = 0;
slot0Configs.kS = 0;
slot0Configs.kV = 0;

CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
currentLimitsConfigs.SupplyCurrentLowerLimit = 0;
currentLimitsConfigs.SupplyCurrentLowerTime = 0;
currentLimitsConfigs.SupplyCurrentLimit = 0;
currentLimitsConfigs.SupplyCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimitEnable = true;
currentLimitsConfigs.StatorCurrentLimit = 0;

FeedbackConfigs feedbackConfigs = new FeedbackConfigs(); 
  feedbackConfigs.FeedbackRemoteSensorID = 0;
  feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; 
  feedbackConfigs.FeedbackRotorOffset = 0;
  feedbackConfigs.RotorToSensorRatio = 0;
  feedbackConfigs.SensorToMechanismRatio = 0;

  motorConf.CurrentLimits = currentLimitsConfigs; 
  motorConf.Slot0 = slot0Configs; 
  motorConf.Feedback = feedbackConfigs; 

  ElevatorR.getConfigurator().apply(motorConf); 
  ElevatorL.getConfigurator().apply(motorConf); 

 

    }
// metodo

public void setPosElevator(double pos ) {
  int rps= 28;
  double cm= 0;
  pos = rps*cm;
ElevatorR.setPosition(pos);
//SmartDashboard.putNumber("position", pos);
  }

    public static ElevatorSub getInstance (){
      if (mElevatorSub== null){
        mElevatorSub= new ElevatorSub();
      }
      return mElevatorSub;
    }

}