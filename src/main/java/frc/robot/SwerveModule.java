package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private PIDController theataController; 
    private double speed; 

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0.0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.ANGLE_OFFSET;//0.155
        this.theataController = new PIDController(Constants.Swerve.ANGLE_KP,0.00, 0);
        this.theataController.enableContinuousInput(-Math.PI, Math.PI);
        this.theataController.setTolerance(0.005);

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.CANCODER_ID, Constants.Swerve.FRC_CANBUS_NAME);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.ANGLE_MOTOR_ID, Constants.Swerve.FRC_CANBUS_NAME);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.DRIVE_MOTOR_ID, Constants.Swerve.FRC_CANBUS_NAME);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

        resetToAbsolute();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        setSpeed(desiredState, isOpenLoop);
        setTheata(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.WHEEL_CIRCUMFERENCE);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
        mDriveMotor.set(0);
    }

    private void setTheata(SwerveModuleState desiredState) {
      //  desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(this.getPosition().angle.getDegrees()));
        theataController.setSetpoint(desiredState.angle.getRadians());
        //SmartDashboard.putNumber(this.moduleNumber + " Sgima ", theataController.calculate(this.getPosition().angle.getRotations()));
        mAngleMotor.set(theataController.calculate(this.getPosition().angle.getRadians()));
        //AngleMotor.set(0);
    }

    private void setTheataControl(SwerveModuleState desiredState) {
       // desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    private void initializeMotors(SwerveModuleConstants moduleConstants) {
        mDriveMotor = new TalonFX(moduleConstants.DRIVE_MOTOR_ID);
        mAngleMotor = new TalonFX(moduleConstants.ANGLE_MOTOR_ID);
        angleEncoder = new CANcoder(moduleConstants.CANCODER_ID);
    }

    private void configureMotors() {
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig); 
    }
}