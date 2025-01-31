/*package frc.robot.subsystems;



import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Brazo extends SubsystemBase {
    private static Brazo mBrazo;
    public SparkMax brazom;
    private SparkClosedLoopController pControler;
       private SparkMaxConfig configBrazo = new SparkMaxConfig();
    public Brazo() {
      brazom = new SparkMax(Constants.MotorConstants.id_mb, MotorType.kBrushless);
      SparkClosedLoopController pController = brazom.getClosedLoopController();
SparkMaxConfig config = new SparkMaxConfig();
config
.inverted(false);

config.closedLoop
    .pid(0.0, 0.0, 0.0)
    .outputRange(-1, 1);

config.limitSwitch
    .forwardLimitSwitchType(Type.kNormallyClosed)
    .forwardLimitSwitchEnabled(true)
    .reverseLimitSwitchType(Type.kNormallyOpen)
    .reverseLimitSwitchEnabled(true);
      
    }
    public void setPos(int rev){
pControler.setReference();
    }
    public static Brazo getInstance (){
        if (mBrazo== null){
          mBrazo = new Brazo();
        }
        return mBrazo;
      } 
}
*/