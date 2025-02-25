package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Brazo extends SubsystemBase {
  private static Brazo mBrazo;
  public SparkMax brazom;
  public SparkClosedLoopController closedLoopController;
  public Encoder EncoderBra= new Encoder(3, 2);

  public RelativeEncoder encoder;
  
  public Drive mDrive;
  double posDrive;
  int sentido;
  public int posb;

  public enum brazoposes {
    none,
    collectCoral,
    collectAlgae,
    nivel1,
    nivel2,
    nivel3,
    nivel4,
    throwAlagae,
    intakeVC;
  }

  public brazoposes brStates;

  public Brazo() {
   
    brStates = brazoposes.none;
    brazom = new SparkMax(Constants.MotorConstants.id_mb, MotorType.kBrushless);
    closedLoopController = brazom.getClosedLoopController();
   encoder= brazom.getEncoder();
   
    
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false);

    config.closedLoop
        .pidf(SmartDashboard.getNumber("BrazoP", 0), SmartDashboard.getNumber("BrazoI", 0), SmartDashboard.getNumber("BrazoD", 0),SmartDashboard.getNumber("BrazoF", 0), ClosedLoopSlot.kSlot0)
       
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    
    config.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyClosed)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);
    config.encoder
        .positionConversionFactor(1)
        
        .velocityConversionFactor(1);

    brazom.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    EncoderBra.setDistancePerPulse(1);
    EncoderBra.setReverseDirection(false);
    // Configures an encoder to average its period measurement over 5 samples
    // Can be between 1 and 127 samples
    EncoderBra.setSamplesToAverage(5);
SmartDashboard.putNumber("EncoderB", encoder.getPosition());
SmartDashboard.putNumber("EncoderBrazo", EncoderBra.getDistance());
  }

  public void setPos(int rev) {
    closedLoopController.setReference(rev, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getPoseB() {
    return encoder.getPosition();
  }

  public static Brazo getInstance() {
    if (mBrazo == null) {
      mBrazo = new Brazo();
    }
    return mBrazo;
  }
  public void setBrazoStates (brazoposes BrazoStates) {
      if (brStates != BrazoStates) {brStates = BrazoStates; }
    }

  public void getposbrazo (double BrazPo){
    brazom.set(BrazPo);
  
  }

  public void setposBra() {
    posDrive = mDrive.GetH();
    SmartDashboard.putNumber("POSICIONDRIVE", posDrive);
    if (posDrive > 0 && posDrive< Math.PI ){
        sentido= 1;
      
      }
    else{
        sentido = -1; 
     }


    switch (brStates) {
      case none:
        setPos(0* sentido);
        posb= 0*sentido;
        break;
      case collectAlgae:
        setPos(0* sentido);
        posb= 0*sentido;

        break;
      case collectCoral:
        setPos(0* sentido);
        posb= 0*sentido;

        break;
      case nivel1:
        setPos(0* sentido);
        posb= 0*sentido;

        break;
      case nivel2:
        setPos(0* sentido);
        posb= 0*sentido;

        break;
      case nivel3:
        setPos(0* sentido);
        posb= 0*sentido;

        break;
      case nivel4:
        setPos(0* sentido);
        posb= 0*sentido;

        break;
      case throwAlagae:
        setPos(0* sentido);
        break;
      default:
        break;
    }
  }

}
