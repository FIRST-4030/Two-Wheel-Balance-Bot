package org.firstinspires.ftc.teamcode;

/**
 * Term class is used to store and process the Design of Experiment Term values
 */
public class Term {
    private double low; // Lowest experiment value
    private double high; // Highest experiment values
    private int n;  // Number of values from Low to High.  Minimum is 2
    private double increment; // Increment value (calculated)
    private double orig; // Place to store the original term value during the experiments
    private double current; // Place to store the current term value during the experiments
    private double min;  // Place to store a minimum response for a degree of freedom
    private double max; // Place to store a minimum response for a degree of freedom

    private double targetValue; // Target Value (not the term value)
    private double valueArea; // Integrated area of the value-target for the experiment time
    public Term(double lowest, double highest, int number, double original) {
        this.low = lowest;
        this.high = highest;
        this.n = number;
        if (n < 2) this.n = 2; // Minimum is 2;
        this.increment = (this.high-this.low)/(this.n-1);
        this.orig = original;
        this.setCurrent(low); // start with the low value
        resetMinMax();
    }

    public int getN() {return this.n;}

    public double getOriginal() {return this.orig;}

    public void next() {
        double check = this.getCurrent() + this.increment;
        if(check > this.high) this.setCurrent(this.low);
        else this.setCurrent(check);
    }

    public void resetMinMax() {
        this.setMin(10000.0);
        this.setMax(-10000.0);
    }
    public void updateMinMax(double value){
        // build the minimum amplitude "box" on the position wave
        this.setMin(Math.min(value, this.getMin()));
        this.setMax(Math.max(value, this.getMax()));
    }
    public double getCurrent() { return current;   }
    public void setCurrent(double current) { this.current = current;    }
    public double getMin() { return min;   }
    public void setMin(double min) { this.min = min; }
    public double getMax() { return max;  }
    public void setMax(double max) { this.max = max;  }
    public void setTargetValue(double value) {this.targetValue = value;}
    public void resetTargetArea() {this.valueArea = 0.0;}
    public void updateTargetArea(double current, double deltaTime) {
        this.valueArea += Math.abs(current-targetValue)*deltaTime;
    }
    public double getValueArea() {return this.valueArea;}
}
