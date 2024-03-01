package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.BeakConstants;
import frc.robot.Constants.NeckExtensionConstants;
import frc.robot.Constants.NeckRotationConstants;

public class NeckSubsystem extends SubsystemBase {

    private final CANSparkMax leftGearbox1;
    private final CANSparkMax leftGearbox2;
    private final CANSparkMax rightGearbox1;
    private final CANSparkMax rightGearbox2;
    private final CANSparkMax neckExtensionMotor;

    private final DigitalInput intakeLimitSwitch;
    private final DigitalInput extensionTopLimitSwitch;
    private final DigitalInput extensionBottomLimitSwitch;
    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    private final Encoder neckEncoder;
    private final DutyCycleEncoder throughBoreEncoder;

    public int neckGoalAngle = 0;
    public int lastNeckGoalAngle = neckGoalAngle;

    public final PIDController neckRotateController = new PIDController(0.0, 0.0, 0.0);
    public final PIDController neckRotateDownInit = new PIDController(0.0, 0.0, 0.0);

    public boolean neckInitialized = false;
    public boolean startedAtTop = false;
    public boolean drivingPositionSet = false;
    public boolean bottomLimitSwitchLastState = false;

    public NeckSubsystem() {
        // Initialize Motor Objects to CAN SparkMAX ID
        leftGearbox1 = NeckRotationConstants.leftGearbox1;
        leftGearbox2 = NeckRotationConstants.leftGearbox2;
        rightGearbox1 = NeckRotationConstants.rightGearbox1;
        rightGearbox2 = NeckRotationConstants.rightGearbox2;
        neckExtensionMotor = NeckExtensionConstants.neckExtensionMotor;

        intakeLimitSwitch = BeakConstants.intakeLimitSwitch;
        extensionTopLimitSwitch = NeckExtensionConstants.extensionTopLimitSwitch;
        extensionBottomLimitSwitch = NeckExtensionConstants.extensionBottomLimitSwitch;
        topLimitSwitch = NeckRotationConstants.topLimitSwitch;
        bottomLimitSwitch = NeckRotationConstants.bottomLimitSwitch;

        neckEncoder = NeckRotationConstants.neckEncoder;
        throughBoreEncoder = NeckExtensionConstants.throughBoreEncoder;
    }

    public void NeckDownInit() {
        double bottomGoal = -290;

        if (!topLimitSwitch.get()) {
            startedAtTop = true;
        }

        if (!neckInitialized && startedAtTop) {
            double p, i, d;

            if (neckEncoder.get() <= 0 && neckEncoder.get() >= -150) {
                p = 0.00125;
                i = 0.0;
                d = 0.0;
            } else if (neckEncoder.get() < -150) {
                p = 0.000725;
                i = 0.00001;
                d = 0.0;
            } else {
                p = 0.0;
                i = 0.0;
                d = 0.0;
            }

            neckRotateDownInit.setPID(p, i, d);
            double neckRotationSpeed = neckRotateDownInit.calculate(neckEncoder.get(), bottomGoal);
            
            if (extensionTopLimitSwitch.get()) {
                ExtendOut();
            } else {
                ExtendStop();
                NeckSetRotateSpeed(neckRotationSpeed);
            }
            
            System.out.println("<-------------------->");
            System.out.println("Speed: " + neckRotationSpeed);
            System.out.println("Encoder Value: " + neckEncoder.get());
            System.out.println("Set Value: " + bottomGoal);

            if (!bottomLimitSwitch.get()) {
                if (bottomLimitSwitchLastState == false) {
                    neckEncoder.reset();
                    bottomLimitSwitchLastState = true;
                }
                neckInitialized = true;
            } else {
                bottomLimitSwitchLastState = false;
            }
        }
    }

    public void IntakePosition() {
        neckGoalAngle = 0;
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void DrivingPosition() {
        neckGoalAngle = 40;
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void SubwooferPosition() {
        neckGoalAngle = 50;
        lastNeckGoalAngle = neckGoalAngle;
    }

    public void AmpPosition() {
        neckGoalAngle = 200;
        lastNeckGoalAngle = neckGoalAngle;
    }

    private void NeckFollower() {
        if (neckInitialized || bottomLimitSwitch.get() == false) {
            updateExtensionAngle();

            if (neckGoalAngle < 30 && neckEncoder.get() >= 30 && (throughBoreEncoder.get() > -1.0 || extensionTopLimitSwitch.get() == true)) {
                // dont go down yet
                double neckRotationSpeed = neckRotateController.calculate(neckEncoder.get(), 50);
                NeckSetRotateSpeed(neckRotationSpeed);
            } else {
                updatePIDConstants();

                double neckRotationSpeed = neckRotateController.calculate(neckEncoder.get(), lastNeckGoalAngle);
                NeckSetRotateSpeed(neckRotationSpeed);
            }
        }
    }

    private void updateExtensionAngle() {
        if (neckGoalAngle == 0 || neckGoalAngle == 200) {
            ExtendOut();
        } else if (neckEncoder.get() > 30 && neckEncoder.get() < 200) {
            ExtendIn();
        } else {
            ExtendStop();
        }
    }

    private void updatePIDConstants() {
        double p, i, d;

        // Calculate PID Constants based on the neck encoder value
        if (neckEncoder.get() <= 30 && neckEncoder.get() > 6 && neckGoalAngle <= 30) {
            // Bottom Controller
            p = 0.003;
            i = 0.00125;
            d = 0.0;
        } else if (neckEncoder.get() >= 190 && neckGoalAngle >= 190) {
            // Top Controller
            p = 0.00005;
            i = 0.00015;
            d = 0.0;
        } else if (neckEncoder.get() > neckGoalAngle - 3 && neckEncoder.get() < neckGoalAngle + 2) {
            // Close Controller
            p = 0.00005;
            i = 0.00075;
            d = 0.0;
        } else if (neckEncoder.get() < neckGoalAngle) {
            // Up Controller
            p = 0.002;
            i = 0.0075;
            d = 0.0002;
        } else if (neckEncoder.get() > neckGoalAngle && neckEncoder.get() > 0) {
            // Down Controller
            p = 0.0016;
            i = 0.0;
            d = 0.0;
        } else {
            // Default Controller
            p = 0.0;
            i = 0.0;
            d = 0.0;
        }

        neckRotateController.setPID(p, i, d);
    }

    public void VisionNeckAngle() {
        boolean hasTargets = LimelightHelpers.getTV("");

        if (hasTargets && LimelightHelpers.getFiducialID("") == 4) {
            Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("");
            
            System.out.println("Z Distance: " + target.getZ());

            neckGoalAngle = visionSetAngle(target.getZ());
            lastNeckGoalAngle = neckGoalAngle;
        }
    }

    private int visionSetAngle(double distance) {
        int angle = 0;
        if (distance > 3.5 && distance < 8) {
            angle = (int) (0.727273*Math.pow(distance, 3) + -15.9913*Math.pow(distance, 2) + 114.026 * distance + -183.197);
        }
        return angle;
    }

    private void NeckSetRotateSpeed(double speed) {
        leftGearbox1.set(speed);
        leftGearbox2.follow(leftGearbox1, false);
        rightGearbox1.follow(leftGearbox1, true);
        rightGearbox2.follow(leftGearbox1, true);

        System.out.println("<-------------------->");
        System.out.println("Encoder Value: " + neckEncoder.get());
        System.out.println("Set Value: " + lastNeckGoalAngle);
        System.out.println("Power: " + speed);
    }


    public void ExtendIn() {
        if (extensionBottomLimitSwitch.get()) {
            neckExtensionMotor.set(0.5);
        } else {
            ExtendStop();
        }
    }

    public void ExtendOut() {
        if (extensionTopLimitSwitch.get()) {
            neckExtensionMotor.set(-0.5);
        } else {
            ExtendStop();
        }
    }

    public void ExtendStop() {
        neckExtensionMotor.set(0.0);
    }

    public void periodic() {
        NeckDownInit();
        NeckFollower();

        if (!intakeLimitSwitch.get()) {
            if (!drivingPositionSet) {
                DrivingPosition();
                drivingPositionSet = true;
            }
        } else {
            drivingPositionSet = false;
        }

        if (!extensionBottomLimitSwitch.get()) {
            throughBoreEncoder.reset();
        }

        if (!bottomLimitSwitch.get()) {
            neckEncoder.reset();
            neckInitialized = true;
        } else {
            if (neckEncoder.get() <= 6 && neckEncoder.get() > 0 && lastNeckGoalAngle == 0) {
                NeckSetRotateSpeed(-0.05);
            }
        }
    }
}
