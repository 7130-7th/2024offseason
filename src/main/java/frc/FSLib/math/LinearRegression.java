package frc.FSLib.math;

public class LinearRegression {

    public static double calculate (double[][] dataSet, double value) {
        return calculate(dataSet, value, 1);
    }

    public static double calculate (double[][] dataSet, double value, int index) {
        int l = 0;
        int r = dataSet.length - 1;
        int mid = (l+r)/2;
        while (l <= r) {
            mid = (l+r)/2;
            if (dataSet[mid][0] > value) r = mid - 1;
            else if (dataSet[mid][0] < value) l = mid + 1;
            else break;
        }
        double dx = value - dataSet[mid-1][0];
        double x = dataSet[mid][0] - dataSet[mid-1][0];
        return dataSet[mid-1][index] * (1-dx/x) + dataSet[mid][index] * dx/x;
    }

    public static final double[][] DISTANCE_TO_ELBOW_AND_SHOOTER = {
        {-Double.MAX_VALUE, -0.243},
        {1.54,-0.22},
        {2.25, -0.1765},
        {3, -0.1564},
        {4, -0.1411},
        {Double.MAX_VALUE, -0.08}
    };

    public static void main(String[] args) {
        System.out.println(calculate(DISTANCE_TO_ELBOW_AND_SHOOTER,1.6 ,1));
    }

}
