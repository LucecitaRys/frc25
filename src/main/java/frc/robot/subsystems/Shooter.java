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




public class Shooter extends SubsystemBase {
  private static Shooter mShooterSubsystem;
   
    private final TalonFX motIk;
    
    private final SparkMax Muneca;
    public int posm;
 
  private TalonFXConfiguration motorConfSho = new TalonFXConfiguration();
  private SparkClosedLoopController closedLoopController;

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
    

    Muneca = new SparkMax(24, MotorType.kBrushless);
    
    motIk = new TalonFX(Constants.MotorConstants.id_mi);
   
    closedLoopController = Muneca.getClosedLoopController();
    
    absoluteEncoderm = Muneca.getAbsoluteEncoder();
  

    SparkMaxConfig configmu = new SparkMaxConfig();
    configmu
        .inverted(false);

    configmu.closedLoop
        .pid(0.5, 0.5, 0.5, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot0);

    configmu.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyClosed)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);
    configmu.absoluteEncoder
        .positionConversionFactor(1)
        .zeroOffset(0)
        .velocityConversionFactor(1);

    Muneca.configure(configmu, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   

    // configuracion kraken
    // Configuración de Límites de Corriente
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLowerLimit = 0.5;
    currentLimitsConfigs.SupplyCurrentLowerTime = 0.5;
    currentLimitsConfigs.SupplyCurrentLimit = 0.5;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = 0.5;

    motorConfSho.CurrentLimits = currentLimitsConfigs;

    motIk.getConfigurator().apply(motorConfSho);

  }


  
  
 public double CurrentL(){
  double current = motIk.getSupplyCurrent().getValueAsDouble();

 return current;
 }
  public void posMuneca(int posmun){
      closedLoopController.setReference(posmun, ControlType.kPosition, ClosedLoopSlot.kSlot0);       
  }
  public void GETPOSESMU(double MUPO){
Muneca.set(MUPO);
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
      motIk.set(constant_vel);
    }
      
    public void setIntakeStates (intake_states intakeStates) {
      if (inStates != intakeStates) {inStates = intakeStates; }
    }

  public void setIntakest(){
    switch (inStates) {
      case none:
      posMuneca(0);
        posm= 0;
        break;
        case collectAlgae:
        posMuneca(0);        
        posm= 0;

        break;
        case collectCoral:
        posMuneca(0);
        posm= 0;

        break;
        case throwCoral:
        motIk.set(.5);
        posm= 0;

        break;
        case throwAlagae:
        posMuneca(0);
        posm= 0;

        break;
        case intakeVC:
        posMuneca(0);
        posm= 0;
        break;
      default:
        break;
      }
    }

  }