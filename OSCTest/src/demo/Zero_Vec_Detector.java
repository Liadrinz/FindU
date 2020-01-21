package demo;

public class Zero_Vec_Detector 
{
	private double[] T_res;
	private int[] zupt;
	public Zero_Vec_Detector(double[] gyrX,double[] gyrY,double[] gyrZ,double[] accX,double[] accY,double[] accZ) 
	{
		zupt = new int[gyrX.length];
		for (int i = 0; i < zupt.length; i++) {
			zupt[i] = 0;
		}
		int W = SimData.Window_size;
		double[] T = new double[gyrX.length];
		T = GLRT(gyrX, gyrY, gyrZ, accX, accY, accZ);
		for (int i = 0; i < T.length; i++) {
			if (T[i]<SimData.gamma) {
				for (int j = i; j < i+W; j++) {
					zupt[j] = 1;
				}
			}
		}
		T_res = new double[(int) (T.length+2*Math.floor(W/2))];
		for (int i = 0; i < Math.floor(W/2); i++) {
			T_res[i]  = Algo.max_a(T);
		}
		for (int i = 0; i < T.length; i++) {
			T_res[i+1] = T[i];
		}
		for (int i = 0; i < Math.floor(W/2); i++) {
			T_res[(int) (i+T_res.length-Math.floor(W/2))]  = Algo.max_a(T);
		}
	}
	public double[] getT() 
	{
		return T_res;
	}
	public int[] getZUPT() 
	{
		return zupt;
	}
	private double[] GLRT(double[] gyrX,double[] gyrY,double[] gyrZ,double[] accX,double[] accY,double[] accZ) {
		double g = SimData.g;
		double sigma2_a = SimData.sigma_a*SimData.sigma_a;
		double sigma2_g = SimData.sigma_g*SimData.sigma_g;
		int W = SimData.Window_size;
		int N = gyrX.length;
		double[] T = new double[N-W+1];
		for (int i = 0; i < T.length; i++) {
			T[i] = 0;
		}
		for (int i = 0; i < T.length-1; i++) {
			double[] ya_m = new double[3];
			int[] index = new int[W];
			for (int j = i; j < i+index.length; j++) {
				index[j-i] = j;
			}
			ya_m[0] = Algo.mean(accX, index);
			ya_m[1] = Algo.mean(accY, index);
			ya_m[2] = Algo.mean(accZ, index);
			for (int j = i; j < i+W ; j++) {
				double[] temp = new double[3];
				temp[0] = accX[j] - g*ya_m[0]/Algo.norm(ya_m);
				temp[1] = accY[j] - g*ya_m[1]/Algo.norm(ya_m);
				temp[2] = accZ[j] - g*ya_m[2]/Algo.norm(ya_m);
				T[i] = T[i]+(gyrX[j]*gyrX[j]+gyrY[j]*gyrY[j]+gyrZ[j]*gyrZ[j])/sigma2_g+(temp[0]*temp[0]+temp[1]*temp[1]+temp[2]*temp[2])/sigma2_a;
			}
		}
		for (int i = 0; i < T.length; i++) {
			T[i]  = T[i]/W;
		}
		return T;
	}
}
