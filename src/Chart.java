import java.awt.BorderLayout;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.ejml.data.DenseMatrix64F;
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
	    
	    XYSeries series1 = new XYSeries("Read Voltage", false);
	    XYSeries series2 = new XYSeries("Filtered Voltage", false);
	    XYSeries series3 = new XYSeries("True Voltage", false);
	    double realValue = 1.5;
	    double measValue;
	    double noise;
	    double mean = 0;
	    double variance = 0.5;
	    
	    // Kalman Filter 
	    DenseMatrix64F F = new DenseMatrix64F(1, 1);
	    DenseMatrix64F Q = new DenseMatrix64F(1, 1);
	    DenseMatrix64F H = new DenseMatrix64F(1, 1);
	    DenseMatrix64F x = new DenseMatrix64F(1, 1);
	    DenseMatrix64F P = new DenseMatrix64F(1, 1);
	    
	    // Model Configuration
	    F.set(0, 1);
	    Q.set(0, 0.00001);
	    H.set(0, 1);
	    
	    // Initial State
	    x.set(0, 3);
	    P.set(0, 1);
	    
	    KalmanFilter myKalman = new KalmanFilterSimple();
	    myKalman.configure(F, Q, H);
	    myKalman.setState(x, P);
	    
	    for(int t=0; t<180; t++) {

	    	// Sensor Readings Voltage
	    	java.util.Random r = new java.util.Random();
	    	noise = r.nextGaussian() * Math.sqrt(variance) + mean;
	    	measValue = realValue + noise;
	    	series1.add(t, measValue);
	    	
	    	
	    	// Filtered Voltage
	    	series2.add(t, myKalman.getState().get(0));
	    	myKalman.update( new DenseMatrix64F(1, 1, true, measValue), new DenseMatrix64F(1, 1, true, 5));
	    	myKalman.predict();
	    	
	    	// True Voltage
	    	series3.add(t, realValue);
	    	
	    }

	    dataset.addSeries(series1);
	    dataset.addSeries(series2);
	    dataset.addSeries(series3);

	    return dataset;	
	}
	
	
}
