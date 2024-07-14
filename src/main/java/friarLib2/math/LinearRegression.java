package friarLib2.math;

public class LinearRegression {

    private double m; // Slope
    private double b; // Y-intercept

    /**
     * Compute m and b
     *
     * <p>I (Zach) copied this code from https://algs4.cs.princeton.edu/code/edu/princeton/cs/algs4/LinearRegression.java.html</p>
     * and don't understand any of it, but it seems to work great!
     *
     * @param data a 2D array or x and y coordinates
     */
    public LinearRegression (double[][] data)
    {
        int n = data.length;

        // First pass
        double sumX = 0.0, sumY = 0.0;
        for (int i = 0; i < n; i++)
        {
            sumX += data[i][0];
            sumY += data[i][1];
        }
        double Xbar = sumX / n;
        double Ybar = sumY / n;

        // Second pass: compute summary statistics
        double XXbar = 0.0, XYbar = 0.0;
        for (int i = 0; i < n; i++)
        {
            XXbar += (data[i][0] - Xbar) * (data[i][0] - Xbar);
            XYbar += (data[i][0] - Xbar) * (data[i][1] - Ybar);
        }
        m = XYbar / XXbar;
        b = Ybar - m * Xbar;
    }

    public double Evaluate (double x)
    {
        return m * x + b;
    }

    public double GetSlope()
    {
        return m;
    }

    public double GetIntercept()
    {
        return b;
    }

}
