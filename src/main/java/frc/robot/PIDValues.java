package frc.robot;

public class PIDValues {
    public double kP;
    public double kI;
    public double kD;
    public double kIz;
    public double kFF;
    public double kMinOutput;
    public double kMaxOutput;

    public PIDValues(double P, double I, double D, double Iz, double FF, double min, double max){
        kP = P;
        kI = I;
        kD = D;
        kIz = Iz;
        kFF = FF;
        kMinOutput = min;
        kMaxOutput = max;
    }



}
