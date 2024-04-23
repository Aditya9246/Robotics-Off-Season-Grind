// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class Util {
    public static class PIDParameters{
        //These are public for the sake of editing during PID tuning (I'm not gonna make 16 getters/setters)
        public final double mKP;
        public final double mKI;
        public final double mKD;
        public final double mKV;
        public final double mKS;
        public final double mContinuousInputMax;
        public final double mContinuousInputMin;
        public final boolean mIsContinuousInput;

        public PIDParameters(double kP, double kI, double kD, double kS, double kV, boolean isContinuousInput, double continuousInputMax, double continuousInputMin){
            mKP = kP;
            mKI = kI;
            mKD = kD;
            mKV = kV;
            mKS = kS;
            mContinuousInputMax = continuousInputMax;
            mContinuousInputMin = continuousInputMin;
            mIsContinuousInput = isContinuousInput;
        }

        public PIDParameters(double kP, double kI, double kD, double kS, double kV){
            this(kP, kI, kD, kS, kV, false, 0,0);
        }

        public PIDParameters(double kP, double kI, double kD){
            this(kP, kI, kD, 0, 0, false, 0,0);
        }
   

        public PIDController toWPIController(){
            PIDController pid = new PIDController(mKP, mKI, mKD);
            if(mIsContinuousInput){pid.enableContinuousInput(mContinuousInputMin, mContinuousInputMax);}
            return pid;
        }

        public Slot0Configs toTalonConfiguration(){
            Slot0Configs config = new Slot0Configs();
            config.kP = mKP;
            config.kI = mKI;
            config.kD = mKD;
            config.kV = mKV;
            config.kS = mKS;
            return config;
        }
        public SimpleMotorFeedforward toWPIMotorFeedForward(){
            return new SimpleMotorFeedforward(mKS, mKV);
        }
    }
    public enum POV{
        DPADUP(0),
        DPADRIGHT(90),
        DPADDOWN(180),
        DPADLEFT(270);

        public final int mAngle;
        private POV(int angle){
            mAngle = angle;
        }
    }
}
