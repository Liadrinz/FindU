package demo;

public class SimData {
	//----------------------- GENERAL PARAMETERS---------------------------------------
	public static double altitude=100;
	public static double latitude=58;
	public static double g = gravity(latitude, altitude);
	public static double sampleperiod = 1.0/250;
	public static double init_heading = 0*Math.PI/180;
	public static double[] init_pos = new double[] {0,0,0}; 
	
	//------------------------Detector Settings---------------------------------------
	public static double sigma_a = 0.01;
	public static double sigma_g = 0.1*Math.PI/180;
	public static int Window_size = 3;
	public static double gamma = 0.3*Math.pow(10, 5);
	//------------------------FILTER PARAMETERS---------------------------------------
	public static String biases = "off";
	public static String scalefactors = "off";
	public static double[] sigma_acc = new double[] {0.5,0.5,0.5};
	public static double[] sigma_gyr = new double[] {0.5*Math.PI/180,0.5*Math.PI/180,0.5*Math.PI/180};
	public static double[] acc_bias_driving_noise = new double[] {0.0000001,0.0000001,0.0000001};
	public static double[] gyro_bias_driving_noise = new double[] {0.0000001*Math.PI/180,0.0000001*Math.PI/180,0.0000001*Math.PI/180};
	public static double[] sigma_vel = new double[] {0.01,0.01,0.01};
	public static double[] sigma_initial_pos = new double[] {Math.pow(10, -5),Math.pow(10, -5),Math.pow(10, -5)};
	public static double[] sigma_initial_vel = new double[] {Math.pow(10, -5),Math.pow(10, -5),Math.pow(10, -5)};
	public static double[] sigma_initial_att = new double[] {0.1*Math.PI/180,0.1*Math.PI/180,0.1*Math.PI/180};
	public static double[] sigma_initial_acc_bias= new double[] {0.3,0.3,0.3};
	public static double[] sigma_initial_gyr = new double[] {0.3*Math.PI/180,0.3*Math.PI/180,0.3*Math.PI/180};
	public static double[] sigma_initial_acc_scale= new double[] {0.0001,0.0001,0.0001};
	public static double[] sigma_initial_gyr_scale= new double[] {0.00001,0.00001,0.00001};
	public static double acc_bias_instability_time_constant_filter = Double.POSITIVE_INFINITY;
	public static double gyr_bias_instability_time_constant_filter = Double.POSITIVE_INFINITY;
	public SimData() {}
	private static double gravity(double lambda,double h) {
		lambda = lambda*Math.PI/180;
		gamma = 9.780327*(1+0.0053024*Math.sin(lambda)*Math.sin(lambda)-0.0000058*Math.sin(2*lambda)*Math.sin(2*lambda));
		return gamma-((3.0877e-6)-(0.004e-6)*Math.sin(lambda)*Math.sin(lambda))*h+(0.072e-12)*h*h;
	}
}
