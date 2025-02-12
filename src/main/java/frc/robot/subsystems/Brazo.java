package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Brazo extends SubsystemBase {
  private static Brazo mBrazo;
  public SparkMax brazom;
  public SparkClosedLoopController closedLoopController;
  public SparkAbsoluteEncoder absoluteEncoder;
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
    absoluteEncoder = brazom.getAbsoluteEncoder();
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false);

    config.closedLoop
        .pidf(0.5, 0.5, 0.5, 0.5, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    ;
    config.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyClosed)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);
    config.absoluteEncoder
        .positionConversionFactor(1)
        .zeroOffset(0)
        .velocityConversionFactor(1);

    brazom.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setPos(int rev) {
    closedLoopController.setReference(rev, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getPoseB() {
    return absoluteEncoder.getPosition();
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

  public void setposBra() {
    posDrive = mDrive.GetH();
    if (posDrive > 0 && posDrive<= 1 ){
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
