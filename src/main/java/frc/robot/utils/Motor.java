package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Motor {
    public enum MotorBrand { Spark, Talon };
    public MotorBrand type;
    public int id;

    public TalonSRX talon;
    public CANSparkMax spark;

    public Motor(int id, MotorBrand type) {
        this.id = id;
        this.type = type;

        if (type == MotorBrand.Spark) {
            spark = new CANSparkMax(id, MotorType.kBrushless);
        } else if (type == MotorBrand.Talon) {
            talon = new TalonSRX(id);
        }
    }

    public void set(double percent) {
        if (type == MotorBrand.Spark) {
            spark.set(percent);
        } else if (type == MotorBrand.Talon) {
            talon.set(ControlMode.PercentOutput, percent);
        }
    }
}
