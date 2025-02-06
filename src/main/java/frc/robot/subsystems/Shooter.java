package frc.robot.subsystems;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.StatusAction;



public class Shooter extends SubsystemBase {
  private static Shooter mShooterSubsystem;
   
    private final TalonFX motIk;
    
    private final SparkMax Muñeca;
    public int posm;
 
  private TalonFXConfiguration motorConfSho = new TalonFXConfiguration();
  private SparkClosedLoopController closedLoopController;
  private SparkClosedLoopController closedLoopControllerGarra;
  private SparkAbsoluteEncoder absoluteEncoderm;
  public enum intake_states {
    none,
    collectCoral,
    collectAlgae,
    throwCoral,
    throwAlagae,
    intakeVC;
  }
  public intake_states inStates = intake_states.none;
  public Shooter() {
    

    Muñeca = new SparkMax(0, MotorType.kBrushless);
    
    motIk = new TalonFX(Constants.MotorConstants.id_mi);
   
    closedLoopController = Muñeca.getClosedLoopController();
    
    absoluteEncoderm = Muñeca.getAbsoluteEncoder();
  

    SparkMaxConfig configmuñ = new SparkMaxConfig();
    configmuñ
        .inverted(false);

    configmuñ.closedLoop
        .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot0);

    configmuñ.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyClosed)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);
    configmuñ.absoluteEncoder
        .positionConversionFactor(1)
        .zeroOffset(0)
        .velocityConversionFactor(1);

    Muñeca.configure(configmuñ, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   

    // configuracion kraken
    // Configuración de Límites de Corriente
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLowerLimit = 0;
    currentLimitsConfigs.SupplyCurrentLowerTime = 0;
    currentLimitsConfigs.SupplyCurrentLimit = 0;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = 0;

    motorConfSho.CurrentLimits = currentLimitsConfigs;

    motIk.getConfigurator().apply(motorConfSho);

  }


  public void VelIntake(double vel){
    
    motIk.set(0.2);
  }
  public void posGarra(double posGar){
     closedLoopControllerGarra.setReference(posGar, ControlType.kPosition, ClosedLoopSlot.kSlot1); 
  }

  public void posMuñeca(int posmuñ){
   
      closedLoopController.setReference(posmuñ, ControlType.kPosition, ClosedLoopSlot.kSlot0);
          
  }
  public double getposm(){
   return  absoluteEncoderm.getPosition();
  }
    public static Shooter getInstance (){
      if (mShooterSubsystem== null){
        mShooterSubsystem = new Shooter();
      }
      return mShooterSubsystem;
    }
  
    
    
  
    public void setConstantVel (double constant_vel) {
      double mConstantVel = constant_vel; 
      motIk.set(mConstantVel);
    }
      

  public void setIntakeStatez(){
    switch (inStates) {
      case none:
        motIk.set(0);
        posm= 0;

        break;
        case collectAlgae:
        motIk.set(.5);
        posm= 0;

        break;
        case collectCoral:
        motIk.set(.5);
        posm= 0;

        break;
        case throwCoral:
        motIk.set(.5);
        posm= 0;

        break;
        case throwAlagae:
        motIk.set(.5);
        posm= 0;

        break;
        case intakeVC:
        motIk.set(.5);
        posm= 0;

        break;
       
      default:
        break;
      }
    }

  }