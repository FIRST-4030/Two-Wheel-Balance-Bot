/*
PID Controller Class, with original source from ChatGPT
Enhancements from SrAmo (utilize rate if provided, getIntegral, Integral reset)
*/

package org.firstinspires.ftc.teamcode;

/**
 * PIDController class.  Proportional, Integral, Derivative Controller Class
 */
public class PIDController {
    private double kp;  // Proportional gain
    private double ki;  // Integral gain
    private double kd;  // Derivative gain

    private double setpoint;  // Desired value
    private double integral;  // Integral sum
    private double previousError;  // Previous error value
    private long lastTime;  // Last timestamp

    /**
     * Constructor for PIDController
     * @param kp proportional gain
     * @param ki integral gain
     * @param kd derivative gain
     */
    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.setpoint = 0.0;
        this.integral = 0.0;
        this.previousError = 0.0;
        this.lastTime = System.currentTimeMillis();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    //public double getSetpoint() { return setpoint;  }

    /**
     * Compute method for PID controller.  Call repeatedly
     */
    public double compute(double processVariable) {
        long now = System.currentTimeMillis();
        double timeChange = (now - lastTime) / 1000.0;  // Convert milliseconds to seconds

        double error = setpoint - processVariable;
        integral += error * timeChange; // accumulate the error over time

        double derivative = (error - previousError) / timeChange;

        double output = kp * error + ki * integral + kd * derivative;

        previousError = error;
        lastTime = now;

        return output;
    }

    /**
     * Compute method with double parameters
     * IMU usually returns angular rates, which can be used rather than deriving the derivative
     * @param processVariable the term for the pit
     * @param derivative the derivative of the term
     */
    public double compute(double processVariable, double derivative) {
        //long now = System.currentTimeMillis();
        //double timeChange = (now - lastTime) / 1000.0;  // Convert milliseconds to seconds
        
        double error = setpoint - processVariable;
        //integral += error * timeChange; // accumulate the error over time
        integral = 0;

        double output = kp * error + ki * integral - kd * derivative;

        previousError = error; 
        //lastTime = now;
        
        return output;
    }

    public void reset() {
        this.integral = 0.0;
        this.previousError = 0.0;
        this.lastTime = System.currentTimeMillis();
    }

    // Getters and setters for gains
    //public double getKp() {   return kp;    }

    //public void setKp(double kp) {    this.kp = kp;    }

    /*
    public double getKi() {
        return ki;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public double getKd() {
        return kd;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    public double getIntegral() {
        return integral;
    }
    */

}
