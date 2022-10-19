package org.firstinspires.ftc.teamcode.Reno.poc;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.time.LocalTime;

public class ConceptPidMotorController extends ConceptMotorController{
    private double Kp = 0.03;                     // factor for "proportional" control
    private double Ki = 0.001;                     // factor for "integral" control
    private double Kd = 0.002;                     // factor for "derivative" control
    private double previousError = 0.0;       // the prior sensor input (used to compute velocity)

    private double error = 0.0;

    private double feedBackValue = 0.0;
    private long previousTime = 0;
    private double goal = 0.0;

    private double proportional = 0.0;
    private double integral = 0.0;
    private double derivative = 0.0;
    private double miniValue = -1.0;
    private double maxValue = 1.0;

    public ConceptPidMotorController(double goal)
    {
        this.goal = goal;
    }
    public ConceptPidMotorController(double Kp, double Ki, double Kd)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }


    public void setFeedbackValue(double value)
    {
        this.feedBackValue = value;
    }
    public void setGoal(double goal)
    {
        this.goal = goal;
    }

    public void reset()
    {
        this.Kp = 0.0;
        this.Ki = 0.0;
        this.Kd = 0.0;
        this.previousTime = 0;
        this.previousError = 0.0;
        this.integral = 0.0;
        this.proportional = 0.0;
        this.derivative = 0.0;
        this.goal = 0.0;

    }
    public double getValue()
    {
        this.error = this.goal - this.feedBackValue;
        long currentTime = System.nanoTime();
        double deltaTime = currentTime - previousTime;
        double deltaError = this.goal - this.previousError;

        proportional = Kp * error;
        integral += error * deltaTime * Ki;

        derivative = 0.0;

        if (deltaTime > 0)
        {
            derivative = (deltaError / deltaTime) * Kd;
        }

        previousError = this.error;
        previousTime = currentTime;

        double pid = proportional + integral + derivative;
        return Range.clip(pid, miniValue, maxValue);

    }
}
