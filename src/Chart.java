import java.awt.BorderLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;

import org.ejml.data.DenseMatrix64F;
import org.ejml.data.Matrix;
import org.ejml.simple.SimpleMatrix;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartFrame;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.data.category.DefaultCategoryDataset;
import org.jfree.data.xy.DefaultXYDataset;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class Chart extends JFrame{

	public Chart() {
		super("XY Line Chart");
		JPanel chartPanel = createChartPanel();
		add(chartPanel, BorderLayout.CENTER);
		
		setSize(640, 480);
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setLocationRelativeTo(null);	
	}
		
	private JPanel createChartPanel() {
		String chartTitle = "My Chart Title";
		String xAxisLabel = "x";
		String yAxisLabel = "y";
		
		XYDataset dataSet = createDataset();
		
		JFreeChart chart = ChartFactory.createXYLineChart(chartTitle, xAxisLabel, yAxisLabel, dataSet);
		
		return new ChartPanel(chart);
	}
	
	private static XYDataset createDataset(){
		XYSeriesCollection dataset = new XYSeriesCollection();
	    
	    XYSeries series1 = new XYSeries("True Voltage", false);
	    XYSeries series2 = new XYSeries("Filtered Voltage", false);
	    XYSeries series3 = new XYSeries("Sensor Readings", false);
	
	    SimpleMatrix measValue = new SimpleMatrix(4, 1);
	    SimpleMatrix realValue = new SimpleMatrix(4, 1);
	
	    double noise = 0;
	    double mean = 0;
	    double variance = 10;
	    
	    double angle = 89;       // The angle from the ground to point cannon
	    double muzzle_vel = 200; // The starting velocity of projectile 
	    
	    SimpleMatrix g = new SimpleMatrix(2, 1);                  // Gravity
	    g.set(0, 0); g.set(1, -9.81);  	                          // Value in X  and Y axis
	    SimpleMatrix velocity = new SimpleMatrix(2, 1);           // Components of cannon velocity
	    velocity.set(0, muzzle_vel*Math.cos(angle*Math.PI/180));  // X axis
	    velocity.set(1, muzzle_vel*Math.sin(angle*Math.PI/180));  // Y axis
	    double dt = 0.01;		                                  // length of intervals of time
	    	    
	    // Initial Values
	    realValue.set(0, 0);				// X pos			
	    realValue.set(1, velocity.get(0));  // X vel
	    realValue.set(2, 100); 				// Y pos
	    realValue.set(3, velocity.get(1));  // Y vel
	    
	    // Kalman Filter 
	    SimpleMatrix F = new SimpleMatrix(4, 4);
	    SimpleMatrix G = new SimpleMatrix(4, 4);
	    SimpleMatrix u = new SimpleMatrix(4, 1);
	    SimpleMatrix Q = new SimpleMatrix(4, 4);
	    SimpleMatrix H = new SimpleMatrix(4, 4);
	    SimpleMatrix P = new SimpleMatrix(4, 4);
	    SimpleMatrix R = new SimpleMatrix(4, 4);
	    
	    // Model Configuration
	    // Matrix F
	    F.set(0, 0, 1);  F.set(0, 1, dt);   F.set(0, 2, 0);  F.set(0, 3, 0);
	    F.set(1, 0, 0);  F.set(1, 1, 1 );   F.set(1, 2, 0);  F.set(1, 3, 0);
	    F.set(2, 0, 0);  F.set(2, 1, 0 );   F.set(2, 2, 1);  F.set(2, 3, dt);
	    F.set(3, 0, 0);  F.set(3, 1, 0 );   F.set(3, 2, 0);  F.set(3, 3, 1);
	    
	    // Matrix G
	    G.set(0, 0, 0);  G.set(0, 1, 0 );   G.set(0, 2, 0);  G.set(0, 3, 0);
	    G.set(1, 0, 0);  G.set(1, 1, 0 );   G.set(1, 2, 0);  G.set(1, 3, 0);
	    G.set(2, 0, 0);  G.set(2, 1, 0 );   G.set(2, 2, 1);  G.set(2, 3, 0);
	    G.set(3, 0, 0);  G.set(3, 1, 0 );   G.set(3, 2, 0);  G.set(3, 3, 1);
	    
	    // Matrix u
	    u.set(0, 0);   u.set(1, 0);   u.set(2, 0.5 * g.get(1) * dt * dt);   u.set(3, g.get(1) * dt);
	    
	    Q.set(0);
	    H.set(SimpleMatrix.identity(4));
	    
	    // Initial State Covariance
	    P.set(SimpleMatrix.identity(4));
	    
	    // Measurement Error Covariance
	    double cov = 5;
	    R.set(0, 0, cov);  R.set(0, 1,    0);  R.set(0,   2,    0);  R.set(0,  3,    0);
	    R.set(1, 0,   0);  R.set(1, 1,  cov);  R.set(1,   2,    0);  R.set(1,  3,    0);
	    R.set(2, 0,   0);  R.set(2, 1,    0);  R.set(2,   2,  cov);  R.set(2,  3,    0);
	    R.set(3, 0,   0);  R.set(3, 1,    0);  R.set(3,   2,    0);  R.set(3,  3,  cov);
	    
	    measValue.set(realValue);
	    
	    // Setting Kalman Filter
    	KalmanFilter myKalman = new KalmanFilterSimple();
	    myKalman.configure(F.getMatrix(), G.getMatrix(), Q.getMatrix(), H.getMatrix());
	    SimpleMatrix initState =  new SimpleMatrix(4, 1);	// Creating a inital state different from real
	    initState = measValue;
	    myKalman.setState(initState.getMatrix(), P.getMatrix());
	   
    	
	    for(double t=0; t<20; t+=dt) {
	    	// True Voltage
	    	series1.add(realValue.get(0), realValue.get(2));	    	
    	
	    	// Filtered Voltage
	    	series2.add(myKalman.getState().get(0), myKalman.getState().get(2));
	    	myKalman.predict(u.getMatrix());
	        myKalman.update(measValue.getMatrix(), R.getMatrix());

	    	// Sensor Readings Voltage
	    	java.util.Random ra = new java.util.Random();
	    	noise = ra.nextGaussian() * Math.sqrt(variance) + mean;
	    	measValue.set(0, realValue.get(0) + noise);
	    	measValue.set(2, realValue.get(2) + noise);
	    	series3.add(measValue.get(0), measValue.get(2));
	    	
	    	// Updating the real value
	    	realValue = F.mult(realValue).plus(G.mult(u));
	    }

	    dataset.addSeries(series1);
	    dataset.addSeries(series2);
	    dataset.addSeries(series3);



	    return dataset;	
	}
	
	
}
