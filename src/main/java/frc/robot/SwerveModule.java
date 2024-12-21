package frc.robot;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;

    private final SimpleMotorFeedforward driveFeedForward;
    private final DutyCycleOut driveDutyCycle;
    private final VelocityVoltage driveVelocity;
    private final PositionVoltage anglePosition;
    private final PIDController theataController;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private Rotation2d angleOffset;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.ANGLE_OFFSET;
        this.driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS,Constants.Swerve.DRIVE_KV,Constants.Swerve.DRIVE_KA);
        this.driveVelocity = new VelocityVoltage(0);
        this.driveDutyCycle = new DutyCycleOut(0);
        this.anglePosition = new PositionVoltage(0);
        this.theataController = new PIDController(moduleNumber, moduleNumber, moduleNumber);
        initializeMotors(moduleConstants);
        configureMotors();
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
    }

    private void setTheata(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(this.getPosition().angle.getRadians()));
        theataController.setSetpoint(desiredState.angle.getRadians());
        mAngleMotor.set(theataController.calculate(this.getPosition().angle.getRadians()));
    }

    private void setTheataControl(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    }

    public void resetToAbsolute() {
        var absolutePosition = getCANcoder().getRadians() - angleOffset.getRadians();
        this.setTheata(new SwerveModuleState(0, new Rotation2d(Units.radiansToDegrees(absolutePosition))));
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
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
