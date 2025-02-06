package frc.robot.subsystems;



import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    private static Vision mVision;
    private final NetworkTable table;
    public Vision (){
table= NetworkTableInstance.getDefault().getTable(getName());
    }
    @Override
    public void periodic (){
      //  double valor1 = 10;
       // table.getEntry("valor").setDouble(valor1); 
    }
    public void salida(double x, double y, double area){
    NetworkTable table1 = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table1.getEntry("tx");
NetworkTableEntry ty = table1.getEntry("ty");
NetworkTableEntry ta = table1.getEntry("ta");

 x = tx.getDouble(0.0);
 y = ty.getDouble(0.0);
 area = ta.getDouble(0.0);

    SmartDashboard.putNumber("tx", x);
    SmartDashboard.putNumber("ty", y);
    SmartDashboard.putNumber("area", area);
}

public static Vision getInstance (){
    if (mVision == null){
      mVision = new Vision();
    }
    return mVision;
  }




}
