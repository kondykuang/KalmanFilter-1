import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;

/**
 * A Kalman filter implemented using SimpleMatrix.  The code tends to be easier to
 * read and write, but the performance is degraded due to excessive creation/destruction of
 * memory and the use of more generic algorithms.  This also demonstrates how code can be
 * seamlessly implemented using both SimpleMatrix and DenseMatrix64F.  This allows code
 * to be quickly prototyped or to be written either by novices or experts.
 *
 * @author Peter Abeles
 */
public class KalmanFilterSimple implements KalmanFilter{
 
    // kinematics description
    private SimpleMatrix F, G, Q, H;
 
    // sytem state estimate
    private SimpleMatrix x, P;
 
    @Override
    public void configure(DenseMatrix64F F,  DenseMatrix64F G, DenseMatrix64F Q, DenseMatrix64F H) {
        this.F = new SimpleMatrix(F); // F = State transition matrix
        this.G = new SimpleMatrix(G); // G = State control matrix
        this.Q = new SimpleMatrix(Q); // Q = Estimated proccess error covariance.
                                      // If you create the proccess (our case) and map
                                      // the equations directly from it, you can assume it as 0.
        this.H = new SimpleMatrix(H); // H = Observation matrix. 
                                      // Multiply a state vector by H to translate it into a measurement vector.
    }
 
    @Override
    public void setState(DenseMatrix64F x, DenseMatrix64F P) {
        this.x = new SimpleMatrix(x);
        this.P = new SimpleMatrix(P);
    }
 
    @Override
    public void predict(DenseMatrix64F _u) {
        SimpleMatrix u = SimpleMatrix.wrap(_u);
        
    	// x = F x  +  G u
        x = (F.mult(x)).plus(G.mult(u));
 
        // P = F P F' + Q
        P = F.mult(P).mult(F.transpose()).plus(Q);
    }
 
    @Override
    public void update(DenseMatrix64F _z, DenseMatrix64F _R) {
        // a fast way to make the matrices usable by SimpleMatrix
        SimpleMatrix z = SimpleMatrix.wrap(_z);
        SimpleMatrix R = SimpleMatrix.wrap(_R);
 
        // y = z - H x
        SimpleMatrix y = z.minus(H.mult(x));
 
        // S = H P H' + R
        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);
 
        // K = PH'S^(-1)
        SimpleMatrix K = P.mult(H.transpose().mult(S.invert()));
 
        // x = x + Ky
        x = x.plus(K.mult(y));
 
        // P = (I-kH)P = P - KHP
        P = P.minus(K.mult(H).mult(P));
    }
 
    @Override
    public DenseMatrix64F getState() {
        return x.getMatrix();
    }
 
    @Override
    public DenseMatrix64F getCovariance() {
        return P.getMatrix();
    }

}