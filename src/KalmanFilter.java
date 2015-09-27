import org.ejml.data.DenseMatrix64F;

public interface KalmanFilter {
	void configure(DenseMatrix64F F, DenseMatrix64F Q, DenseMatrix64F H);
	public void setState(DenseMatrix64F x, DenseMatrix64F P);
	public void predict();
	public void update(DenseMatrix64F _z, DenseMatrix64F _R);
	public DenseMatrix64F getState();
	public DenseMatrix64F getCovariance();

}
