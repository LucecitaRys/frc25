package frc.lib.swerve;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Drive;
import frc.robot.Constants.SwerveModules;
import frc.lib.Conversions;
import frc.robot.Telemetry;
import frc.robot.Constants.Drive.DriveControlMode;

public class SwerveModule {
    private final int moduleNumber;
    private final TalonFX mSteerMotor, mDriveMotor;
    private final CANcoder mCANcoder; 
    
    private PeriodicIO mPeriodicIO = new PeriodicIO(); 
    private ModuleState targetModuleState; 
    

    public SwerveModule (SwerveModuleConstants moduleConstants, int moduleNumber){ 
        this.moduleNumber = moduleNumber; 
        mSteerMotor = new TalonFX(moduleConstants.steerMotorID); 
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mCANcoder = new CANcoder(moduleConstants.cancoderID); 
        
        //SteerMotor Config
        ClosedLoopGeneralConfigs closedLoopConfigs = new ClosedLoopGeneralConfigs(); 
        closedLoopConfigs.ContinuousWrap = true; 
        TalonFXConfiguration steer_config = new TalonFXConfiguration();
        steer_config.ClosedLoopGeneral = closedLoopConfigs;
        steer_config.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(45).withSupplyCurrentLowerLimit(50).withSupplyCurrentLowerTime(0.1).withSupplyCurrentLimitEnable(true);
        steer_config.Feedback = new FeedbackConfigs().withFeedbackRemoteSensorID(mCANcoder.getDeviceID()).withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder).withRotorToSensorRatio(SwerveModules.steering_gear_ratio);
        steer_config.Slot0 = new Slot0Configs().withKP(SwerveModules.steer_kP).withKI(SwerveModules.steer_kI).withKD(SwerveModules.steer_kD).withKS(SwerveModules.steer_kS); 
        mSteerMotor.getConfigurator().apply(steer_config); 
        //DriveMotor Config
        TalonFXConfiguration drive_config = new TalonFXConfiguration(); 
        drive_config.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(50).withSupplyCurrentLowerLimit(70).withSupplyCurrentLowerTime(0.1).withSupplyCurrentLimitEnable(true); 
        drive_config.Feedback = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor).withSensorToMechanismRatio(SwerveModules.drive_gear_ratio); 
        drive_config.Slot0 = new Slot0Configs().withKP(SwerveModules.drive_kP).withKI(SwerveModules.drive_kI).withKD(SwerveModules.drive_kD).withKS(SwerveModules.drive_kS); 
        mDriveMotor.getConfigurator().apply(drive_config); 
        //CANcoder Config
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        // cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = AbsoluteSensorDiscontinuityPoint.Signed_PlusMinusHalf; 
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; 
        cancoderConfigs.MagnetSensor.MagnetOffset = moduleConstants.angleOffset; 
        mCANcoder.getConfigurator().apply(cancoderConfigs);
        //Configs Update 
        setNeutralMode(false); 
    }

    public static class PeriodicIO {
        //Inputs
        double currentAngle = 0;
        double velocity = 0; 
        double drivePosition = 0;
        //Outputs
        double rotationDemand = 0;
        double driveDemand = 0; 
        DriveControlMode controlMode = DriveControlMode.Velocity; 
    }

    public static class SwerveModuleConstants {
        public final int driveMotorID;
        public final int steerMotorID;
        public final int cancoderID;
        public final double angleOffset;

        public SwerveModuleConstants(int driveMotorID, int steerMotorID, int cancoderID, double angleOffset) {
            this.driveMotorID = driveMotorID;
            this.steerMotorID = steerMotorID;
            this.cancoderID = cancoderID;
            this.angleOffset = angleOffset;
        }
    }
    
    public void readPeriodicInputs (){
        var absPos= mCANcoder.getAbsolutePosition();
        var velocity = mDriveMotor.getVelocity(); 
        var position = mDriveMotor.getPosition(); 
        BaseStatusSignal.refreshAll(absPos, velocity, position); 
        mPeriodicIO.currentAngle = absPos.getValueAsDouble();
        mPeriodicIO.velocity = velocity.getValueAsDouble() * SwerveModules.wheelCircumference; 
        mPeriodicIO.drivePosition = position.getValue().in(Degrees); 
       SmartDashboard.putNumber("absPos", absPos.getValue().in(Degrees)); 
        SmartDashboard.putNumber("velocity", velocity.getValueAsDouble()); 
        SmartDashboard.putNumber("position", position.getValueAsDouble()); 
        
    }

   
    public void writePeriodicOutputs (){
        if (targetModuleState == null) {
            return;
        }
      
       SwerveModuleState moduleStateOptimized = SwerveModuleState.optimize(
            new SwerveModuleState(targetModuleState.speedMetersPerSecond, targetModuleState.angle), 
            Rotation2d.fromRotations(mPeriodicIO.currentAngle)
        );

        //SwerveModuleState moduleStateOptimized = new SwerveModuleState(targetModuleState.speedMetersPerSecond, targetModuleState.angle);
        //Rotation2d.fromRotations(mPeriodicIO.currentAngle);

      //  SwerveModuleState moduleStateOptimized =  new SwerveModuleState(targetModuleState.speedMetersPerSecond, targetModuleState.angle), Rotation2d.fromRotations(mPeriodicIO.currentAngle);
        

     
        double targetVelocity = moduleStateOptimized.speedMetersPerSecond; 
        Rotation2d targetAngleRot = moduleStateOptimized.angle; 
        mPeriodicIO.rotationDemand = targetAngleRot.getRotations(); 
        mSteerMotor.setControl(new PositionDutyCycle(mPeriodicIO.rotationDemand)); 
        
        if (mPeriodicIO.controlMode == DriveControlMode.Velocity){
            mPeriodicIO.driveDemand = Conversions.MPSToRPS(targetVelocity, SwerveModules.wheelCircumference, SwerveModules.drive_gear_ratio)*60; 
            mDriveMotor.setControl(new VelocityDutyCycle(mPeriodicIO.driveDemand)); 
        } else {
            mPeriodicIO.driveDemand = targetVelocity / Drive.maxVelocity;
            mDriveMotor.setControl(new DutyCycleOut(mPeriodicIO.driveDemand)); 
        }
    }

    public void resetModule (){
        mDriveMotor.setPosition(0);  
    }

    public void setModuleState (ModuleState desiredModuleState, DriveControlMode controlMode) { 
        targetModuleState = desiredModuleState; 
        mPeriodicIO.controlMode = controlMode; 
    }

    public ModuleState getModuleState (){
        return new ModuleState(mPeriodicIO.drivePosition, Rotation2d.fromDegrees(mPeriodicIO.currentAngle), mPeriodicIO.velocity); 
    }

    public void setNeutralMode (boolean wantBrake){
        MotorOutputConfigs neutralMode = new MotorOutputConfigs();
        neutralMode.NeutralMode = wantBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mSteerMotor.getConfigurator().apply(neutralMode);
        mDriveMotor.getConfigurator().apply(neutralMode); 
    }

    public void setDriveControlMode (DriveControlMode mode){
        mPeriodicIO.controlMode = DriveControlMode.PercentOutput; 
    }

    public void outputTelemetry (){
        ShuffleboardLayout motorsData = Telemetry.mSwerveTab.getLayout("Module " + moduleNumber, BuiltInLayouts.kList)
        .withSize(2, 3).withPosition(2 * moduleNumber, 0);
        motorsData.addDouble("Desired Angle", () -> targetModuleState.angle.getDegrees());
        motorsData.addDouble("Current Angle", () -> Rotation2d.fromRotations(mPeriodicIO.currentAngle).getDegrees()); 
        motorsData.addDouble("Desired Velocity", () -> targetModuleState.speedMetersPerSecond);  
        motorsData.addDouble("Current Velocity", ()-> mPeriodicIO.velocity); 
    }
}
