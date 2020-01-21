package demo;

import java.io.BufferedWriter;

public class ZUPTaidedINS {
	private double[][] x_h = null,cov=null,P = null,Q = null,R = null,H = null,id = null,F=null,G=null;
	private double[] quat=null;
	public ZUPTaidedINS(double[] gyrX,double[] gyrY,double[] gyrZ,double[] accX,double[] accY,double[] accZ,int[] zupt) {
		// TODO Auto-generated constructor stub
		int N = gyrX.length;
		P = new double[9][9];
		Q = new double[6][6];
		H = new double[3][9];
		R = new double[3][3];
		F = new double[9][9];
		G = new double[9][6];
		cov =new double[9][N];
		x_h = new double[9][N];
		id = new double[9][9];
		quat =new double[4];
		init_filter(P, Q, R, H);
		init_vec(x_h, cov, id, N, P);
		init_Nav_eq(x_h, quat, gyrX, gyrY, gyrZ, accX, accY, accZ);
		for (int i = 1; i < N; i++) {
//-------------------------------TIME UPDATE----------------------------------------------------
			double[] x_h_k1 = new double[9];
			for (int j = 0; j < x_h_k1.length; j++) {
				x_h_k1[j] = x_h[j][i];
			}
			double[] u_h = comp_imu_errors(gyrX[i], gyrY[i], gyrZ[i], accX[i], accY[i], accZ[i], x_h_k1);
			Navigation_equations(x_h, quat, u_h,i-1);
			state_matrix(F, G, quat, u_h);
			P =Mat.plus(Mat.multi(Mat.multi(F, P),Mat.zhz(F)), Mat.multi(Mat.multi(G, Q),Mat.zhz(G))) ;
			P =Mat.divide(Mat.plus(P, Mat.zhz(P)), 2) ;
			for (int j = 0; j < cov.length; j++) {
				cov[j][i] = P[j][j];
			}
//-------------------------------ZERO-VEL UPDATE--------------------------------------------------
			if (zupt[i]==1) {
				double[][] K = Mat.divide(Mat.multi(P, Mat.zhz(H)), Mat.plus( Mat.multi(Mat.multi(H, P), Mat.zhz(H)), R));
				double[]z = new double[3];
				z[0] = -x_h[3][i];
				z[1] = -x_h[4][i];
				z[2] = -x_h[5][i];
				double[]dx = Mat.multi(K, z);
				double[]x_hk = new double[9];
				for (int j = 0; j < x_hk.length; j++) {
					x_hk[j] = x_h[j][i];
				}
				comp_internal_states(x_hk, dx, quat);
				for (int j = 0; j < x_hk.length; j++) {
					x_h[j][i]=x_hk[j];
				}
				P=Mat.multi(Mat.minus(id, Mat.multi(K, H)), P);
				P =Mat.divide(Mat.plus(P, Mat.zhz(P)), 2) ;
				for (int j = 0; j < cov.length; j++) {
					cov[j][i] = P[j][j];
				}
			}
		}
	}
	public double[][] getX_h() {
		return x_h;
	}
	public double[][] getCov()
	{
		return cov;
	}
	private void init_filter(double[][]P,double[][]Q,double[][]R,double[][]H) {
		for (int i = 0; i < P.length; i++) {
			for (int j = 0; j < P.length; j++) {
				P[i][j] = 0;
			}
		}
		for (int i = 0; i < Q.length; i++) {
			for (int j = 0; j < Q.length; j++) {
				Q[i][j] = 0;
			}
		}
		for (int i = 0; i < H.length; i++) {
			for (int j = 0; j < H[0].length; j++) {
				H[i][j] = 0;
			}
		}
		H[0][3] = 1;
		H[1][4] = 1;
		H[2][5] = 1;
		P[0][0] = SimData.sigma_initial_pos[0]*SimData.sigma_initial_pos[0];
		P[1][1] = SimData.sigma_initial_pos[1]*SimData.sigma_initial_pos[1];
		P[2][2] = SimData.sigma_initial_pos[2]*SimData.sigma_initial_pos[2];
		P[3][3] = SimData.sigma_initial_vel[0]*SimData.sigma_initial_vel[0];
		P[4][4] = SimData.sigma_initial_vel[1]*SimData.sigma_initial_vel[1];
		P[5][5] = SimData.sigma_initial_vel[2]*SimData.sigma_initial_vel[2];
		P[6][6] = SimData.sigma_initial_att[0]*SimData.sigma_initial_att[0];
		P[7][7] = SimData.sigma_initial_att[1]*SimData.sigma_initial_att[1];
		P[8][8] = SimData.sigma_initial_att[2]*SimData.sigma_initial_att[2];
		Q[0][0] = SimData.sigma_acc[0]*SimData.sigma_acc[0];
		Q[1][1] = SimData.sigma_acc[1]*SimData.sigma_acc[1];
		Q[2][2] = SimData.sigma_acc[2]*SimData.sigma_acc[2];
		Q[3][3] = SimData.sigma_gyr[0]*SimData.sigma_gyr[0];
		Q[4][4] = SimData.sigma_gyr[1]*SimData.sigma_gyr[1];
		Q[5][5] = SimData.sigma_gyr[2]*SimData.sigma_gyr[2];
		R[0][0] = SimData.sigma_vel[0]*SimData.sigma_vel[0];
		R[1][1] = SimData.sigma_vel[1]*SimData.sigma_vel[1];
		R[2][2] = SimData.sigma_vel[2]*SimData.sigma_vel[2];
	}
	private void init_vec(double[][]x_h,double[][] cov,double[][] id,int N,double[][]P) {
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < N; j++) {
				cov[i][j] = 0;
				x_h[i][j] = 0;
			}
		}
		for (int i = 0; i < P.length; i++) {
			for (int j = 0; j < P.length; j++) {
				if (j==i) {
					id[i][j] = 1;
				}else {
					id[i][j] = 0;
				}
			}
		}
		for (int i = 0; i < cov.length; i++) {
			cov[i][0] = P[i][i];
		}
	}
	private void init_Nav_eq(double[][]x_h,double[] quat,double[] gyrX,double[] gyrY,double[] gyrZ,double[] accX,double[] accY,double[] accZ) {
		int[] index = new int[20];
		for (int i = 0; i < index.length; i++) {
			index[i] = i;
		}
		double f_u = Algo.mean(accX, index);
		double f_v = Algo.mean(accY, index);
		double f_w = Algo.mean(accZ, index);
		double roll = Math.atan2(-f_v, -f_w);
		double pitch = Math.atan2(f_u, Math.sqrt(f_v*f_v+f_w*f_w));
		double[] attitude = new double[] {roll,pitch,SimData.init_heading};
		double[][] Rb2t = Mat.zhz(Rt2b(attitude));
		double[] quat1 = dcm2q(Rb2t);
		for (int i = 0; i < quat1.length; i++) {
			quat[i]  = quat1[i];
		}
		x_h[0][0] = SimData.init_pos[0];
		x_h[1][0] = SimData.init_pos[1];
		x_h[2][0] = SimData.init_pos[2];
		x_h[3][0] = 0;
		x_h[4][0] = 0;
		x_h[5][0] = 0;
		x_h[6][0] = attitude[0];
		x_h[7][0] = attitude[1];
		x_h[8][0] = attitude[2];
	}
	double[] comp_imu_errors(double gyrX,double gyrY,double gyrZ,double accX,double accY,double accZ,double[] x_h) {
		double[] u_h = new double[6];
		u_h[0] = accX;
		u_h[1] = accY;
		u_h[2] = accZ;
		u_h[3] = gyrX;
		u_h[4] = gyrY;
		u_h[5] = gyrZ;
		return u_h;
	}
	private void Navigation_equations(double[][]x_h,double[] quat,double[] u_h,int k_1){
		double[] y = new double[] {0,0,0,0,0,0,0,0,0};
		double Ts = SimData.sampleperiod;
		double[] w_tb = new double[] {u_h[3],u_h[4],u_h[5]};
		double P = w_tb[0]*Ts;
		double Q = w_tb[1]*Ts;
		double R = w_tb[2]*Ts;
		double[][] OMEGA = new double[][] {{0,R*0.5,-Q*0.5,P*0.5},{-R*0.5,0,P*0.5,Q*0.5},{Q*0.5,-P*0.5,0,R*0.5},{-P*0.5,-Q*0.5,-R*0.5,0}};
		double v = Algo.norm(w_tb)*Ts;
		if (v!=0) {
			double[] quat1 = Mat.multi(Mat.plus(Mat.multi(Math.cos(v/2), Mat.I(4)),Mat.multi(2/v*Math.sin(v/2), OMEGA)), quat);
			for (int i = 0; i < quat.length; i++) {
				quat[i] = quat1[i]/Algo.norm(quat1);
			}
		}
		double[][] Rb2t = q2dcm(quat);
		y[6] = Math.atan2(Rb2t[2][1], Rb2t[2][2]);
		y[7] = -Math.atan(Rb2t[2][0]/Math.sqrt(1-Rb2t[2][0]*Rb2t[2][0]));
		y[8] = Math.atan2(Rb2t[1][0], Rb2t[0][0]);
		double[] g_t = new double[] {0,0,SimData.g};
		double[] u_h_13 = new double[] {u_h[0],u_h[1],u_h[2]};
		double[] f_t = Mat.multi(q2dcm(quat), u_h_13);
		double[] acc_t = Mat.plus(f_t, g_t);
		double[][]A = Mat.I(6);
		A[0][3] = Ts;
		A[1][4] = Ts;
		A[2][5] = Ts;
		double[][]B = new double[][] {{(Ts*Ts)/2,0,0},{0,(Ts*Ts)/2,0},{0,0,(Ts*Ts)/2},{Ts,0,0},{0,Ts,0},{0,0,Ts}};
		double[] x_h_16 = new double[] {x_h[0][k_1],x_h[1][k_1],x_h[2][k_1],x_h[3][k_1],x_h[4][k_1],x_h[5][k_1]};
		double[] y_16 = Mat.plus(Mat.multi(A, x_h_16), Mat.multi(B, acc_t));
		x_h[0][k_1+1] = y_16[0];
		x_h[1][k_1+1] = y_16[1];
		x_h[2][k_1+1] = y_16[2];
		x_h[3][k_1+1] = y_16[3];
		x_h[4][k_1+1] = y_16[4];
		x_h[5][k_1+1] = y_16[5];
		x_h[6][k_1+1] = y[6];
		x_h[7][k_1+1] = y[7];
		x_h[8][k_1+1] = y[8];
	}
	private void state_matrix(double[][] F,double[][]G,double[] quat,double[]u_h) {
		double[][] Rb2t = q2dcm(quat);
		double[] u_h_13 = new double[] {u_h[0],u_h[1],u_h[2]};
		double[] f_t = Mat.multi(Rb2t, u_h_13);
		double[][] St = new double[][] {{0,-f_t[2],f_t[1]},{f_t[2],0,-f_t[0]},{-f_t[1],f_t[0],0}};
		//double[] O = new double[] {0,0,0};
		double[][] I = Mat.I(3);
//		double[][] Da = new double[][] {{u_h[0],0,0},{0,u_h[1],0},{0,0,u_h[2]}};
//		double[][] Dg = new double[][] {{u_h[3],0,0},{0,u_h[4],0},{0,0,u_h[5]}};
		double[][] Fc = new double[9][9];
		for (int j = 0; j < 9; j++) {
			for (int j2 = 0; j2 < 9; j2++) {
				Fc[j][j2] = 0;
			}
		}
		for (int j2 = 0; j2 < I.length; j2++) {
			for (int k = 0; k < I.length; k++) {
				Fc[j2][k+3] = I[j2][k];
			}
		}
		for (int j2 = 0; j2 < St.length; j2++) {
			for (int k = 0; k < St.length; k++) {
				Fc[j2+3][k+6] = St[j2][k];
			}
		}
		double[][] Gc = new double[9][6];
		for (int j = 0; j < 9; j++) {
			for (int j2 = 0; j2 < 6; j2++) {
				Gc[j][j2] = 0;
			}
		}
		for (int j2 = 0; j2 < Rb2t.length; j2++) {
			for (int k = 0; k < Rb2t.length; k++) {
				Gc[j2+3][k] = Rb2t[j2][k];
			}
		}
		for (int j2 = 0; j2 < Rb2t.length; j2++) {
			for (int k = 0; k < Rb2t.length; k++) {
				Gc[j2+6][k+3] = -Rb2t[j2][k];
			}
		}
		double[][] F1 = Mat.plus(Mat.I(9), Mat.multi(SimData.sampleperiod,Fc));
		double[][] G1 = Mat.multi(SimData.sampleperiod, Gc);
		for (int j = 0; j < F1.length; j++) {
			for (int j2 = 0; j2 < F1[0].length; j2++) {
				F[j][j2] = F1[j][j2];
			}
		}
		for (int j = 0; j < G1.length; j++) {
			for (int j2 = 0; j2 < G1[0].length; j2++) {
				G[j][j2] = G1[j][j2];
			}
		}
	}
	private void comp_internal_states(double[] x_hk,double[]dx,double[]quat) {
		double[][] R = q2dcm(quat);
		double[] x_out = Mat.plus(x_hk, dx);
		double[] epsilon = new double[] {dx[6],dx[7],dx[8]};
		double[][] OMEGA = new double[][] {{0,-epsilon[2],epsilon[1]},{epsilon[2],0,-epsilon[0]},{-epsilon[1],epsilon[0],0}};
		R = Mat.multi(Mat.minus(Mat.I(3), OMEGA), R);
		x_out[6] = Math.atan2(R[2][1], R[2][2]);
		x_out[7] = -Math.atan(R[2][0]/Math.sqrt(1-R[2][0]*R[2][0]));
		x_out[8] = Math.atan2(R[1][0], R[0][0]);
		for (int i = 0; i < x_hk.length; i++) {
			x_hk[i] = x_out[i];
		}
		double[] quat1 = dcm2q(R);
		for (int i = 0; i < quat1.length; i++) {
			quat[i] = quat1[i];
		}
	}
	private double[][] q2dcm(double[] q) {
		double[] p = new double[6];
		for (int i = 0; i < 4; i++) {
			p[i] = q[i]*q[i];
		}
		p[4] = p[1]+p[2];
		double temp = p[0]+p[3]+p[4];
		if ((p[0]+p[3]+p[4])!=0) {
			p[5] = 2.0/temp;
		}
		else {
			p[5] = 0;
		}
		double[][]R =new double[3][3];
		R[0][0] = 1-p[5]*p[4];
		R[1][1] = 1-p[5]*(p[0]+p[2]);
		R[2][2] = 1-p[5]*(p[0]+p[1]);
		p[0] = p[5]*q[0];
		p[1] = p[5]*q[1];
		p[4] = p[5]*q[2]*q[3];
		p[5] = p[0]*q[1];
		
		R[0][1]=p[5]-p[4];
		R[1][0]=p[5]+p[4];

		p[4]=p[1]*q[3];
		p[5]=p[0]*q[2];

		R[0][2]=p[5]+p[4];
		R[2][0]=p[5]-p[4];

		p[4]=p[0]*q[3];
		p[5]=p[1]*q[2];

		R[1][2]=p[5]-p[4];
		R[2][1]=p[5]+p[4];
		return R;
	}
	private double[] dcm2q(double[][] R) {
		double T = 1+R[0][0]+R[1][1]+R[2][2];
		double S,qw,qx,qy,qz;
		if (T>1e-8) {
			 S = 0.5 /Math.sqrt(T);
			 qw = 0.25/S;
			 qx = ( R[2][1] - R[1][2] ) * S;
			 qy = ( R[0][2] - R[2][0] ) * S;
			 qz = ( R[1][0] - R[0][1] ) * S;
		}else {
			if (R[0][0]>R[1][1]&&R[0][0]>R[2][2]) {
				 S = Math.sqrt(1+R[0][0]-R[1][1]-R[2][2]);
				 qw = ( R[2][1] - R[1][2] ) * S;
				 qx = 0.25*S;
				 qy = ( R[0][1] + R[1][0] ) * S;
				 qz = ( R[0][2] + R[2][0] ) * S;
			}else if (R[1][1]>R[2][2]) {
				 S = Math.sqrt(1+R[1][1]-R[0][0]-R[2][2]);
				 qw = ( R[0][2] - R[2][0] ) * S;
				 qx =( R[0][1] + R[1][0] ) * S;
				 qy =  0.25*S;
				 qz = ( R[1][2] + R[2][1] ) * S;
			}else {
				S = Math.sqrt(1+R[2][2]-R[0][0]-R[1][1]);
				 qw = ( R[1][0] - R[0][1] ) * S;
				 qx = ( R[0][2] + R[2][0] ) * S;
				 qy = ( R[1][2] + R[2][1] ) * S;
				 qz =0.25*S;
			}
		}
		double[] q = new double[]{qx,qy,qz,qw};
		return q;
	}
	private double[][] Rt2b(double[] ang) {
		double cr = Math.cos(ang[0]);
		double sr = Math.sin(ang[0]);
		double cp = Math.cos(ang[1]);
		double sp = Math.sin(ang[1]);
		double cy = Math.cos(ang[2]);
		double sy = Math.sin(ang[2]);
		double[][]R =new double[][] {{cy*cp,sy*cp,-sp},{-sy*cr+cy*sp*sr,cy*cr+sy*sp*sr,cp*sr},{sy*sr+cy*sp*cr,-cy*sr+sy*sp*cr,cp*cr}};
		return R;
	}
}
