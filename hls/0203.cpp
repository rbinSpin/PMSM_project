#include "matmult.h"
#include "hls_stream.h"
#include <math.h>

void Add_3d(float MatA[3][3], float MatB[3][3], float MatC[3][3]);
void Add_4d(float MatA[4][4], float MatB[4][4], float MatC[4][4]);
void Multiply_3d(float MatA[3][3], float MatB[3][3], float MatC[3][3]);
void Multiply_4d(float MatA[4][4], float MatB[4][4], float MatC[4][4]);
void Swap(float &A, float &B);
void Saturate(float A, float upper_limit, float lower_limit, float &B);
void Gaussian_Elimination_3d(float MatrixA[3][3], float MatrixB[3][3], float Minv[3][3]);
void Gaussian_Elimination_4d(float MatrixA[4][4], float MatrixB[4][4], float Minv[4][4]);
void Gaussian_Elimination_T_3d(float MatrixA[3][3], float MatrixB[3][3], float Minv[3][3]);
void Gaussian_Elimination_T_4d(float MatrixA[4][4], float MatrixB[4][4], float Minv[4][4]);
void Transpose_3d(float MatA[3][3], float MatB[3][3]);
void Transpose_4d(float MatA[4][4], float MatB[4][4]);

//void CAL_idsd(float K12, float iqs, float &idsd);
//void CAL_iqsd_temp(float K1, float K11, float ids, float idsd, float &iqsd_temp);
//void CAL_iqsd(float K2, float Omega_d, float K3, float t_L, float K11, float idsd, float iqs, float iqsd_temp, float &iqsd);
//void CAL_Vqs(float K4, float iqs, float iqsd, float K5, float Omega_d, float K10, float ids, float idsd, float Omega, float lq, float u_1, float u_2, float u_3, float &Vqs);
//void CAL_Vds(float K7, float ids, float idsd, float K9, float iqs, float iqsd, float Omega_d, float Omega, float ld, float u_1, float u_2, float u_3, float &Vds);


template <typename T> void sda_main(T a[N2], T out[N2])
{
	const int r_con = 925;		// 2
//    const int step_con = 7;		// 13
    const int r_obs = 320;		// 2.5
//    const int step_obs = 9;	// 15
    
//    const int r_con = 1750;
    const int step_con = 6;
//    const int r_obs = 540;
    const int step_obs = 8;

//    float invR_con[2][2] = {{1000, 0},
//	   	       	   	   	    {0, 1000}};

//    G = B*inv(R)*Transpose(B)
    float G_con[3][3] = {{0,0,0},
					     {0,976562.5,0},
					     {0,0,3086419.75}};

//    float G_con[3][3] = {0};

    float As_con[3][3] = {0};
    float Gs_con[3][3] = {0};
    float Hs_con[3][3] = {0};

//    float G_con_temp[3][2] = {{0,0},
//    						  {31250.0,0},
//							  {0,55555.5556}};

    // float inv_Ar_T_con[3][3] = {0};
    float Init_Var1_con[3][3] = {0};
    float Init_Var2_con[3][3] = {0};
    float Init_Var3_con[3][3] = {0};
	float Init_Var4_con[3][3] = {0};
	float tempM1_con[3][3] = {0};
    float tempM2_con[3][3] = {0};
	float tempM3_con[3][3] = {0};
	float As_con_new[3][3] = {0};
	float GsHs_con[3][3] = {0};
	float IGsHs_con[3][3] = {0};
	float As_T_Hs_con[3][3] = {0};
	float As_T_con[3][3] = {0};
	float Gs_con_new_temp[3][3] = {0};
	float Gs_con_new[3][3] = {0};
	float G_inv_T_Ar_con[3][3] = {0};
	float inv_Ar_T_Q_con[3][3] = {0};
	float Hs_con_new_temp[3][3] = {0};
	float Hs_con_new[3][3] = {0};
	float Ar_con[3][3] = {0};
	float Ar_T_con[3][3] = {0};
	float As_new_con_temp[3][3] = {0};

	float Var1_con[3][3] = {0};
	float Var2_con[3][3] = {0};
	
	float Identity_3d[3][3] = {{1,0,0},
							   {0,1,0},
							   {0,0,1}};
	float Identity_4d[4][4] = {{1,0,0,0},
							   {0,1,0,0},
							   {0,0,1,0},
							   {0,0,0,1}};

//    float invR_obs[3][3] = {0};

//	G = B*inv(R)*Transpose(B)
	float G_obs[4][4] = {{0, 0, 0, 0},
						 {0, 100000, 0, 0},
					     {0, 0, 100000, 0},
				   	     {0, 0, 0, 100000}};

    float As_obs[4][4] = {0};
    float Gs_obs[4][4] = {0};
    float Hs_obs[4][4] = {0};

//    float G_obs_temp[4][3] = {{0, 0, 0},
//							  {100000, 0, 0},
//							  {0, 100000, 0},
//							  {0, 0, 100000}};

    // float inv_Ar_T_obs[4][4] = {0};
    float Init_Var1_obs[4][4] = {0};
    float Init_Var2_obs[4][4] = {0};
    float Init_Var3_obs[4][4] = {0};
	float Init_Var4_obs[4][4] = {0};
	float tempM1_obs[4][4] = {0};
    float tempM2_obs[4][4] = {0};
	float tempM3_obs[4][4] = {0};
	// float invAr_obs[4][4] = {0};
	float As_obs_new[4][4] = {0};
	float GsHs_obs[4][4] = {0};
	float IGsHs_obs[4][4] = {0};
	float As_T_obs[4][4] = {0};
	float As_T_Hs_obs[4][4] = {0};
	float Gs_obs_new_temp[4][4] = {0};
	float Gs_obs_new[4][4] = {0};
	float G_inv_T_Ar_obs[4][4] = {0};
	float inv_Ar_T_Q_obs[4][4] = {0};
	float Hs_obs_new_temp[4][4] = {0};
	float Hs_obs_new[4][4] = {0};
	float Ar_obs[4][4] = {0};
	float Ar_T_obs[4][4] = {0};
	float As_new_obs_temp[4][4] = {0};

	float Var1_obs[4][4] = {0};
	float Var2_obs[4][4] = {0};

	float u_obs[4][3] = {0};

//	Bc_obs*invR_obs
	const float u_temp_obs[4][3] = {{0, 0, 0},
							  	    {100000, 0, 0},
									{0, 100000, 0},
									{0, 0, 100000}};

	// p=4;
	// Rs = 2.48;
	// Ld = 74.98e-3;
 	// Lq = 113.91e-3;
 	// J = 4.2e-4;
 	// B_value = 1e-4;
 	// Lambdam = 0.193;

	// p = 6  ;                 // Number of poles
	// Rs = 2.5 ;               // Resistance of motor [Ohm]
	const float Ld = 0.018;             // Inductance of motor [H]
	const float Lq = 0.032;             // Inductance of motor [H]
	// J = 7.82e-4 ;            // Equivalent inertia [kg.m2]
	// B_value = 0.02277 ;      // Viscous Friction Coefficient (N*m*s/rad)
	// Lambdam = 0.30175 ;      // Magnetic Flux Linkage (V*s/rad)

	float omega_est = 0, iqs_est = 0, ids_est = 0;
	float Vqs = 0, Vds = 0;
	float delta_t = 0.00001;
	float omega_d = 0;

	float T_L = 0 , omega = 0, iqs = 0, ids = 0;
	float T_L_dot = 0 , omega_est_dot = 0, iqs_est_dot = 0, ids_est_dot = 0;
	float T_L_new = 0, omega_est_new = 0, iqs_est_new = 0, ids_est_new = 0;
	float T_L_out = 0, omega_est_out = 0, iqs_est_out = 0, ids_est_out = 0;

	float dot_obs[4];
	float dot_out[4];

//	float A_obs [4][4] = {0};

//	float Bc_obs[4][3] = {{0, 0, 0},
//					   	   {1, 0, 0},
//					   	   {0, 1, 0},
//					   	   {0, 0, 1}};

	// Bc_T_obs = Transpose(B)
//	float Bc_T_obs[3][4] = {{0, 1, 0, 0},
//						    {0, 0, 1, 0},
//						    {0, 0, 0, 1}};

	float Q_obs[4][4] = {{0.1, 0, 0, 0},
					 	 {0, 100, 0, 0},
						 {0, 0, 100, 0},
						 {0, 0, 0, 100}};      //Qo

//	float R_obs[3][3] = {{0.00001, 0, 0},
//			   	   	       {0, 0.00001, 0},
//					       {0, 0, 0.00001}};

//	float invR_obs[3][3] = {{100000, 0, 0},
//			   	   	        {0, 100000, 0},
//					        {0, 0, 100000}};

//	float A_con[3][3] = {0};
//	float Bc_con[3][2] = {0};
//	float Bc_T_con[2][3] = {0};

	float Q_con[3][3] = {{1, 0, 0},
						 {0, 5, 0},
						 {0, 0, 5}};      //Qc

//	float R_con[2][2] = {{0.001, 0},
//			   	   	       {0, 0.001}};
    
	float u_con[2][3] = {0};

//	invR_con*Bc_T_con
	const float u_temp_con[2][3] = {{0,-31250.0,0},
							   	    {0,0,-55555.5556}};

	float delta_obs[3] = {0,0,0};

	float delta_con[3] = {0,0,0};

	float idsd = 0, iqsd = 0, iqsd_temp = 0;

//	float omega_temp = 0, omega_d_temp = 0;

	float Vqs_in = 0, Vds_in = 0;
	float Vqs_out = 0, Vds_out = 0;

	const float R2r = 0.31415926;  //0.314159265358979

	const float k1 = 5209.23913;		// k1 = 3*p*p/2/J/4*Lambdam ;
	const float k2 = 29.117647;			// k2 = B_value/J ;
	const float k3 = 3836.31713;		// k3 = p/2/J ;
	const float k4 = 78.125;			// k4 = Rs/Lq ;
	const float k5 = 9.4296875;			// k5 = Lambdam/Lq ;
	const float k6 = 31.25;				// k6 = 1/Lq ;
	const float k7 = 138.8888889;		// k7 = Rs/Ld ;
	const float k8 = 55.55555566;		// k8 = 1/Ld ;
	const float k9 = 1.777777778;		// k9 = Lq/Ld ;
	const float k10 = 0.5625;			// k10 = Ld/Lq ;
	const float k11 = -241.68797953;	// k11 = 3*p*p/2/J/4*(Ld-Lq) ;
	const float k12 = -0.0463960;		// k12 = (Ld-Lq)/Lambdam ;
			
//////////////////////	input  //////////////////////
	T_L = a[0];
	iqs_est = a[1];
	ids_est = a[2];
	omega_est = a[3];	// RPM*0.314159265358979

	T_L_dot = a[4];
	iqs_est_dot = a[5];
	ids_est_dot = a[6];
	omega_est_dot = a[7];

	// add Vqs, Vds
	Vqs_in = a[8];
	Vds_in = a[9];

	omega = a[10];		// RPM
//	omega_temp = a[10] ;		// RPM
	iqs = a[11];
	ids = a[12];
	omega_d = a[13];	// RPM
//	omega_d_temp = a[13] ;	// RPM
	delta_t = a[14];


//////////////////////	input  //////////////////////

/////////////////////initialize output/////////////////////
	out[0] = 0;
	out[1] = 0;
	out[2] = 0;
	out[3] = 0;
	out[4] = 0;
	out[5] = 0;
	out[6] = 0;
	out[7] = 0;
	out[8] = 0;
	out[9] = 0;
	out[10] = 0;
	out[11] = 0;
	out[12] = 0;
	out[13] = 0;
	out[14] = 0;
	out[15] = 0;
/////////////////////initialize output/////////////////////

//	omega = omega_temp*R2r;
//	omega_d = omega_d_temp*R2r;

//	A_obs[1][0] = -k3;
//	A_obs[1][1] = -k2;
//	A_obs[1][2] = k1;
//	A_obs[1][3] = k11*iqs_est;
//	A_obs[2][1] = -k5-k10*ids_est;
//	A_obs[2][2] = -k4;
//	A_obs[3][1] = k9*iqs_est;
//	A_obs[3][3] = -k7;

	// Ar_obs = Ar = A-r*I
	Ar_obs[0][0] = -r_obs;
	Ar_obs[0][1] = -k3;
	Ar_obs[1][1] = -k2-r_obs;
	Ar_obs[2][1] = k1;
	Ar_obs[3][1] = k11*iqs_est;
	Ar_obs[1][2] = -k5-k10*ids_est;
	Ar_obs[2][2] = -k4-r_obs;
	Ar_obs[1][3] = k9*iqs_est;
	Ar_obs[3][3] = -k7-r_obs;

	Transpose_4d(Ar_obs, Ar_T_obs);

	// Ar_T_obs[0][0] = -r_obs;
	// Ar_T_obs[0][1] = -k3;
	// Ar_T_obs[1][1] = -k2-r_obs;
	// Ar_T_obs[2][1] = k1;
	// Ar_T_obs[3][1] = k11*iqs_est;
	// Ar_T_obs[1][2] = -k5-k10*ids_est;
	// Ar_T_obs[2][2] = -k4-r_obs;
	// Ar_T_obs[1][3] = k9*iqs_est;
	// Ar_T_obs[3][3] = -k7-r_obs;

//	A_con[0][0] = -k2;
//	A_con[0][1] = k1;
//	A_con[0][2] = k11*iqs;
//	A_con[1][0] = -k5-k10*ids;
//	A_con[1][1] = -k4;
//	A_con[2][0] = k9*iqs;
//	A_con[2][2] = -k7;

//	Bc_con[1][0] = k6;
//	Bc_con[2][1] = k8;

	// Bc_T_con = Transpose(B)
//	Bc_T_con[0][1] = k6;
//	Bc_T_con[1][2] = k8;


	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Observer
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	T_L_new = T_L_dot*delta_t + T_L;                  //	Only this will be passed to the controller
	omega_est_new = omega_est_dot*delta_t + omega_est;
	iqs_est_new = iqs_est_dot*delta_t + iqs_est;
	ids_est_new = ids_est_dot*delta_t + ids_est;

//  estimated state saturation
	Saturate(T_L_new, 4.8, -4.8, T_L_out);
	Saturate(omega_est_new, 471.238898, -471.238898, omega_est_out);
	Saturate(iqs_est_new, 4.8, -4.8, iqs_est_out);
	Saturate(ids_est_new, 4.8, -4.8, ids_est_out);


	// controller init:
	idsd = k12*iqs*iqs;

	delta_con[2] = ids - idsd;

	iqsd_temp = 1/(k1 + k11*delta_con[2]);
	iqsd = (k2*omega_d + k3*T_L_out - k11*idsd*iqs)*iqsd_temp;

	delta_con[1] = iqs - iqsd;
	delta_con[0] = omega - omega_d;
#pragma HLS ARRAY_PARTITION variable=delta_con type=complete

	// Ar_con = Ar = A-r*I
	Ar_con[0][0] = -k2-r_con;
	Ar_con[0][1] = k1;
	Ar_con[0][2] = k11*delta_con[1];
	Ar_con[1][0] = -k5-k10*delta_con[2];
	Ar_con[1][1] = -k4-r_con;
	Ar_con[2][0] = k9*delta_con[1];
	Ar_con[2][2] = -k7-r_con;

	Transpose_3d(Ar_con, Ar_T_con);


	// init:
	// Transpose(Ar)X=H -> inv(Transpose(Ar))*H
	Gaussian_Elimination_4d(Ar_T_obs, Q_obs, inv_Ar_T_Q_obs);
//	for(int i=0; i<4; i++)
//	{
//		for(int j=0; j<4; j++)
//		{
//			Init_Var3_obs[i][j] = inv_Ar_T_Q_obs[i][j];
//		}
//	}
	
	// G*inv(Transpose(Ar))*H
	Multiply_4d(G_obs, inv_Ar_T_Q_obs, Init_Var1_obs);

	// Ar+G*inv(Transpose(Ar))*H
	Add_4d(Ar_obs, Init_Var1_obs, Init_Var2_obs);

	// inv(Ar+G*inv(Transpose(Ar))*H)
	Gaussian_Elimination_4d(Init_Var2_obs, Identity_4d, tempM1_obs);

	// Transpose(Ar+G*inv(Transpose(Ar))*H)
	Transpose_4d(Init_Var2_obs, Init_Var3_obs);

	// Transpose(inv(Transpose(Ar))*H)
	Transpose_4d(inv_Ar_T_Q_obs, Init_Var4_obs);

	// inv(Transpose(Ar))*H*inv(Ar+G*inv(Transpose(Ar))*H)
	Gaussian_Elimination_T_4d(Init_Var3_obs, Init_Var4_obs, tempM2_obs);

//	for(int i=0; i<4; i++)
//	{
//		for(int j=0; j<4; j++)
//		{
//			Init_Var4_obs[i][j] = tempM1_obs[i][j];
//		}
//	}
//	Multiply_4d(Init_Var3_obs, Init_Var4_obs, tempM2_obs);

	// G*inv(Transpose(Ar))
	Gaussian_Elimination_T_4d(Ar_obs, G_obs, G_inv_T_Ar_obs);

	// inv(Ar+G*inv(Transpose(Ar))*H)*G*inv(Transpose(Ar))
	Gaussian_Elimination_4d(Init_Var2_obs, G_inv_T_Ar_obs, tempM3_obs);
//	Multiply_4d(tempM1_obs, G_inv_T_Ar_obs, tempM3_obs);

	for(int i=0; i<4; i++){
		for(int j=0; j<4; j++){
			As_obs[i][j] = Identity_4d[i][j] + 2*r_obs*tempM1_obs[i][j];
			Hs_obs[i][j] = 2*r_obs*tempM2_obs[i][j];
			Gs_obs[i][j] = 2*r_obs*tempM3_obs[i][j];
		}
	}


	//////////// loop till step_obs ////////////////////////////////////////////////////////////////////////////////
	for(int loop_obs = 0; loop_obs < step_obs; loop_obs++)
	{
		// Gs*Hs
		Multiply_4d(Gs_obs, Hs_obs, GsHs_obs);

		//	I+Gs*Hs
		Add_4d(GsHs_obs, Identity_4d, IGsHs_obs);


		////////// As_new = As*inv(I+Gs*Hs)*As /////////////////////////////////////////////////////////////////////////////
		// inv(I+Gs*Hs)*As
		Gaussian_Elimination_4d(IGsHs_obs, As_obs, As_new_obs_temp);

		// As*inv(I+Gs*Hs)*As
		Multiply_4d(As_obs, As_new_obs_temp, As_obs_new);


		////////// Gs_new = Gs + As*inv(I+Gs*Hs)*Gs*Transpose(As) //////////////////////////////////////////////////////////
		// Transpose(As)
		Transpose_4d(As_obs, As_T_obs);

		// inv(I+Gs*Hs)*Gs
		Gaussian_Elimination_4d(IGsHs_obs, Gs_obs, Var1_obs);

		// inv(I+Gs*Hs)*Gs*Transpose(As)
		Multiply_4d(Var1_obs, As_T_obs, Var2_obs);

		// As*inv(I+Gs*Hs)*Gs*Transpose(As)
		Multiply_4d(As_obs, Var2_obs, Gs_obs_new_temp);

		// Gs + As*Gs*inv(I+Hs*Gs)*Transpose(As)
		Add_4d(Gs_obs, Gs_obs_new_temp, Gs_obs_new);


		////////// Hs_new = Hs + Transpose(As)*Hs*inv(I+Gs*Hs)*As //////////////////////////////////////////////////////////
		// Transpose(As)*Hs
		Multiply_4d(As_T_obs, Hs_obs, As_T_Hs_obs);

		// Transpose(As)*Hs*inv(I+Gs*Hs)*As
		Multiply_4d(As_T_Hs_obs, As_new_obs_temp, Hs_obs_new_temp);

		// Hs + Transpose(As)*inv(I+Hs*Gs)*Hs*As
		Add_4d(Hs_obs, Hs_obs_new_temp, Hs_obs_new);


		////////// update As,Gs,Hs ///////////////////////////////////////////////////////////////////////

		for (int i = 0; i < 4; i++){
#pragma HLS UNROLL
			for (int j = 0; j < 4; j++){
				As_obs[i][j] = As_obs_new[i][j];
				Gs_obs[i][j] = Gs_obs_new[i][j];
				Hs_obs[i][j] = Hs_obs_new[i][j];
			}
		}
	}

	// Hs is the solve of SDA: P


	/////////////////////////////////////////////////////////////////////////////////////////////////////


	for (int i=0; i < 4; i++)
	{
//#pragma HLS UNROLL
		for (int j=0; j < 3; j++)
		{
			float temp = 0;
			for (int k=0; k < 4; k++)
			{
				temp += Hs_obs[i][k]*u_temp_obs[k][j];
			}
			u_obs[i][j] = temp;
		}
	}


//	/////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma HLS ARRAY_PARTITION variable=u_obs type=complete

//	T_L_dot = u_obs[0][0] * (omega - omega_est_out) + u_obs[0][1] * (iqs - iqs_est_out) + u_obs[0][2] * (ids - ids_est_out);
//	omega_est_dot = u_obs[1][0] * (omega - omega_est_out) + u_obs[1][1] * (iqs - iqs_est_out) + u_obs[1][2] * (ids - ids_est_out)
//					+ k1*iqs_est_out -k2*omega_est_out - k3*T_L_out + k11*iqs_est_out*ids_est_out;
//	iqs_est_dot = u_obs[2][0] * (omega - omega_est_out) + u_obs[2][1] * (iqs - iqs_est_out) + u_obs[2][2] * (ids - ids_est_out)
//					- k4*iqs_est_out - k5*omega_est_out + k6*Vqs_in - k10*omega_est_out*ids_est_out;
//	ids_est_dot = u_obs[3][0] * (omega - omega_est_out) + u_obs[3][1] * (iqs - iqs_est_out) + u_obs[3][2] * (ids - ids_est_out)
//					- k7*ids_est_out + k9*omega_est_out*iqs_est_out + k8*Vds_in;



	delta_obs[0] = omega - omega_est_out;
	delta_obs[1] = iqs - iqs_est_out;
	delta_obs[2] = ids - ids_est_out;

	for(int i=0; i<4; i++)
	{
		float temp = 0;
		for(int j=0; j<3; j++)
		{
			temp += u_obs[i][j]*delta_obs[j];
		}
		dot_obs[i] = temp;
	}

	// dot_out = [TL, w, iqs, ids]

	dot_out[0] = dot_obs[0];
	dot_out[1] = dot_obs[1] + (k1*iqs_est_out - k2*omega_est_out - k3*T_L_out + k11*iqs_est_out*ids_est_out);
	dot_out[2] = dot_obs[2] + (-k4*iqs_est_out - k5*omega_est_out + k6*Vqs_in - k10*omega_est_out*ids_est_out);
	dot_out[3] = dot_obs[3] + (-k7*ids_est_out + k9*omega_est_out*iqs_est_out + k8*Vds_in);


	///////////////////////////The above is the Observer///////////////////////////
	///////////////////////////The above is the Observer///////////////////////////
	///////////////////////////The above is the Observer///////////////////////////
	///////////////////////////The above is the Observer///////////////////////////
	///////////////////////////The above is the Observer///////////////////////////






	/////////////////////////////////////////////////////////////////////////////////////////////////////
	//                    Controller
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	//////////////////////////// Controller SDA /////////////////////////////////////////////////////////

	// init:
	// inv(Transpose(Ar))*H
	Gaussian_Elimination_3d(Ar_T_con, Q_con, inv_Ar_T_Q_con);
//	for(int i=0; i<3; i++)
//	{
//		for(int j=0; j<3; j++)
//		{
//			Init_Var3_con[i][j] = inv_Ar_T_Q_con[i][j];
//		}
//	}

	// G*inv(Transpose(Ar))*H
	Multiply_3d(G_con, inv_Ar_T_Q_con, Init_Var1_con);

	// Ar+G*inv(Transpose(Ar))*H
	Add_3d(Ar_con, Init_Var1_con, Init_Var2_con);

	// inv(Ar+G*inv(Transpose(Ar))*H)
	Gaussian_Elimination_3d(Init_Var2_con, Identity_3d, tempM1_con);

	// Transpose(Ar+G*inv(Transpose(Ar))*H)
	Transpose_3d(Init_Var2_con, Init_Var3_con);

	// Transpose(inv(Transpose(Ar))*H)
	Transpose_3d(inv_Ar_T_Q_con, Init_Var4_con);

	// inv(Transpose(Ar))*H*inv(Ar+G*inv(Transpose(Ar))*H)
	Gaussian_Elimination_T_3d(Init_Var3_con, Init_Var4_con, tempM2_con);

//	for(int i=0; i<3; i++)
//	{
//		for(int j=0; j<3; j++)
//		{
//			Init_Var4_con[i][j] = tempM1_con[i][j];
//		}
//	}
//	Multiply_3d(Init_Var3_con, Init_Var4_con, tempM2_con);

	// G*inv(Transpose(Ar))
	Gaussian_Elimination_T_3d(Ar_con, G_con, G_inv_T_Ar_con);

	// inv(Ar+G*inv(Transpose(Ar))*H)*G*inv(Transpose(Ar))
	Gaussian_Elimination_3d(Init_Var2_con, G_inv_T_Ar_con, tempM3_con);
//	Multiply_3d(tempM1_con, G_inv_T_Ar_con, tempM3_con);


	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			As_con[i][j] = Identity_3d[i][j] + 2*r_con*tempM1_con[i][j];
			Hs_con[i][j] = 2*r_con*tempM2_con[i][j];
			Gs_con[i][j] = 2*r_con*tempM3_con[i][j];
		}
	}


	//////////// loop till step_con //////////////////////////////////////////////////////////////////////
	for(int loop_con = 0; loop_con < step_con; loop_con++)
	{
		// Gs*Hs
		Multiply_3d(Gs_con, Hs_con, GsHs_con);

		//	I+Gs*Hs
		Add_3d(GsHs_con, Identity_3d, IGsHs_con);


		////////// As_new = As*inv(I+Gs*Hs)*As /////////////////////////////////////////////////////////////////////////////
		// inv(I+Gs*Hs)*As
		Gaussian_Elimination_3d(IGsHs_con, As_con, As_new_con_temp);

		// As*inv(I+Gs*Hs)*As
		Multiply_3d(As_con, As_new_con_temp, As_con_new);


		////////// Gs_new = Gs + As*inv(I+Gs*Hs)*Gs*Transpose(As) //////////////////////////////////////////////////////////
		// Transpose(As)
		Transpose_3d(As_con, As_T_con);

		// inv(I+Gs*Hs)*Gs
		Gaussian_Elimination_3d(IGsHs_con, Gs_con, Var1_con);

		// inv(I+Gs*Hs)*Gs*Transpose(As)
		Multiply_3d(Var1_con, As_T_con, Var2_con);

		// As*inv(I+Gs*Hs)*Gs*Transpose(As)
		Multiply_3d(As_con, Var2_con, Gs_con_new_temp);

		// Gs + As*Gs*inv(I+Hs*Gs)*Transpose(As)
		Add_3d(Gs_con, Gs_con_new_temp, Gs_con_new);


		////////// Hs_new = Hs + Transpose(As)*Hs*inv(I+Gs*Hs)*As //////////////////////////////////////////////////////////
		// Transpose(As)*Hs
		Multiply_3d(As_T_con, Hs_con, As_T_Hs_con);

		// Transpose(As)*Hs*inv(I+Gs*Hs)
		Multiply_3d(As_T_Hs_con, As_new_con_temp, Hs_con_new_temp);

		// Hs + Transpose(As)*inv(I+Hs*Gs)*Hs*As
		Add_3d(Hs_con, Hs_con_new_temp, Hs_con_new);


		////////// update As,Gs,Hs ///////////////////////////////////////////////////////////////////////

		for(int m = 0; m < 3; m++){
#pragma HLS UNROLL
			for(int n = 0; n < 3; n++){
				As_con[m][n] = As_con_new[m][n];
				Gs_con[m][n] = Gs_con_new[m][n];
				Hs_con[m][n] = Hs_con_new[m][n];
			}
		}
	}
	
	// Hs_con is the solve of SDA: P


///////////////////////////////////////////////////////////////////////////////////////////////////// 

	for(int i=0; i < 2 ; i++)            //2 times
	{
//#pragma HLS UNROLL
		for(int j=0; j < 3 ; j++)        //3 times
		{
			float temp = 0;
			for(int k=0; k < 3 ; k++)    //3 times
			{
				temp += u_temp_con[i][k]*Hs_con[k][j];
			}
			u_con[i][j] = temp;
		}
	}
//	Multiply_3d(u_temp_con, Hs_con, u_con);

#pragma HLS ARRAY_PARTITION variable=u_con type=complete
	float Vs[2];

	for(int i=0; i<2; i++)
	{
		float temp = 0;
		for(int j=0;j<3;j++)
		{
			temp += u_con[i][j]*delta_con[j];
		}
		Vs[i] = temp;
	}

	Vqs = Vs[0] + (k4*iqsd + k5*omega_d + k10*delta_con[2]*omega_d + k10*omega*idsd)*Lq;
	Vds = Vs[1] + (k7*idsd - k9*delta_con[1]*omega_d - k9*omega*iqsd)*Ld;


//	float Vqs_comp = (k4*iqsd + k5*omega_d + k10*delta_con[0]*omega_d + k10*omega*idsd)* Lq;
//	float Vds_comp = (k7*idsd - k9*delta_con[1]*omega_d - k9*omega*iqsd)* Ld;

//	Vqs = Vqs_comp + (u_con[0][0] * delta_con[2] + u_con[0][1] * delta_con[1] + u_con[0][2] * delta_con[0]);
//	Vds = Vds_comp + (u_con[1][0] * delta_con[2] + u_con[1][1] * delta_con[1] + u_con[1][2] * delta_con[0]);



//	CAL_idsd(k12, iqs, idsd);
//	CAL_iqsd_temp(k1, k11, ids, idsd, iqsd_temp);
//	CAL_iqsd(k2, omega_d, k3, T_L_new, k11, idsd, iqs, iqsd_temp, iqsd);
//	CAL_Vqs(k4, iqs, iqsd, k5, omega_d, k10, ids, idsd, omega, Lq, u_con[0][0], u_con[0][1], u_con[0][2], Vqs);
//	CAL_Vds(k7, ids, idsd, k9, iqs, iqsd, omega_d, omega, Ld, u_con[1][0], u_con[1][1], u_con[1][2], Vds);

	///////////////////////////The above is the Controller///////////////////////////
	///////////////////////////The above is the Controller///////////////////////////
	///////////////////////////The above is the Controller///////////////////////////
	///////////////////////////The above is the Controller///////////////////////////
	///////////////////////////The above is the Controller///////////////////////////

	float inv_Volt = 0.004234; // 1/236.14
	Saturate(Vqs*inv_Volt, 0.3, -0.3, Vqs_out);
	Saturate(Vds*inv_Volt, 0.1, -0.1, Vds_out);

	out[0] = T_L_out;
	out[1] = iqs_est_out;
	out[2] = ids_est_out;
	out[3] = omega_est_out;
	out[4] = dot_out[0];
	out[5] = dot_out[2];
	out[6] = dot_out[3];
	out[7] = dot_out[1];
	out[8] = Vqs_out;
	out[9] = Vds_out;

	return;
}




extern "C" {
	void Riccati_Solver(hls::stream<axis_t> &in, hls::stream<axis_t> &out) {
//	#pragma HLS INTERFACE s_axilite port = return bundle = control
	#pragma HLS INTERFACE ap_ctrl_none port = return
	#pragma HLS INTERFACE axis port = in
	#pragma HLS INTERFACE axis port = out

	  DataType l_A[N2];
	  DataType l_C[N2];

	#pragma HLS ARRAY_PARTITION variable = l_A factor = 16 dim = 1 cyclic
	#pragma HLS ARRAY_PARTITION variable = l_C factor = 16 dim = 1 cyclic

	  int j_limit = 16; // 512 / DataTypeSize
//	  int i_limit = 1; // N2 / j_limit
	  converter_t converter_load[16];
	  converter_t converter_write[16];

	load_A:
		axis_t temp = in.read();
		for(int j = 0; j < j_limit; j++) {
			#pragma HLS UNROLL
			int high = j * DataTypeSize + DataTypeSize - 1;
			int low = j * DataTypeSize;

			converter_load[j].i = temp.data.range(high, low);
			l_A[j] = converter_load[j].d;
		}

		sda_main<DataType>(l_A, l_C);

	writeC:
		axis_t W_temp;
		for(int j = 0; j < j_limit; j++) {
			#pragma HLS UNROLL
			int high = j * DataTypeSize + DataTypeSize - 1;
			int low = j * DataTypeSize;
			converter_write[j].d = l_C[j];
			W_temp.data.range(high, low) = converter_write[j].i;
		}

		W_temp.last = 1;
		W_temp.keep = -1; // enabling all bytes
		out.write(W_temp);
	}
}




//void CAL_idsd(float K12, float iqs, float &idsd)
//{
//	idsd = K12*iqs*iqs;
//	return;
//}
//void CAL_iqsd_temp(float K1, float K11, float ids, float idsd, float &iqsd_temp)
//{
//	iqsd_temp = 1/(K1 + K11*(ids-idsd));
//	return;
//}
//void CAL_iqsd(float K2, float Omega_d, float K3, float t_L, float K11, float idsd, float iqs, float iqsd_temp, float &iqsd)
//{
//	iqsd = (K2*Omega_d + K3*t_L - K11*idsd*iqs)*iqsd_temp;
//	return;
//}
//void CAL_Vqs(float K4, float iqs, float iqsd, float K5, float Omega_d, float K10, float ids, float idsd, float Omega, float lq, float u_1, float u_2, float u_3, float &Vqs)
//{
//	Vqs = (K4*iqsd + K5*Omega_d + K10*(ids - idsd)*Omega_d + K10*Omega*idsd)* lq +
//		      (u_1 * (Omega-Omega_d) + u_2 * (iqs-iqsd) + u_3 * (ids-idsd));
//	return;
//}
//void CAL_Vds(float K7, float ids, float idsd, float K9, float iqs, float iqsd, float Omega_d, float Omega, float ld, float u_1, float u_2, float u_3, float &Vds)
//{
//	Vds = (K7*idsd - K9*(iqs - iqsd)*Omega_d - K9*Omega*iqsd)* ld +
//		     (u_1 * (Omega-Omega_d) + u_2 * (iqs-iqsd) + u_3 * (ids-idsd));
//	return;
//}


void Gaussian_Elimination_3d(float MatrixA[3][3], float MatrixB[3][3], float Minv[3][3])
{
	int j;
	float localVariable, localVariable_copy, localVariable2, localVariable2_copy;
	float MatA[3][3] = {0};
	float MatB[3][3] = {0};

	for(int m=0; m<3; m++)
	{
		for(int n=0; n<3; n++)
		{
			MatA[m][n] = MatrixA[m][n];
			MatB[m][n] = MatrixB[m][n];
		}
	}

	for(int k=0; k<3; k++)
	{
		// if diagonal is zero, then return if all row elements are 0
        if(MatA[k][k]==0)
        {
        	j = k + 1;
        	while(MatA[j][k] == 0)
        	{
                if(j >= 2)
                {
    				return;
                }
                else
                {
            		j = j + 1;
                }
			}

            for(int x=k; x < 3; x++)
            {
				#pragma HLS UNROLL
            	Swap(MatA[k][x], MatA[j][x]);
			}

            for(int x=0; x < 3; x++)
            {
				#pragma HLS UNROLL
            	Swap(MatB[k][x], MatB[j][x]);
			}
        }


        localVariable = 1/MatA[k][k];
		localVariable_copy = localVariable;
		MatA[k][k] = 1;

		for(int m=k+1; m<3; m++)
		{
			#pragma HLS UNROLL
			MatA[k][m]*=localVariable;
		}

		for(int m=0; m<3; m++)
		{
			#pragma HLS UNROLL
			MatB[k][m]*=localVariable_copy;
		}


	 	for(int m=0; m<3; m++)
	 	{
	 		if(m != k)
			{
				localVariable2 = MatA[m][k];
				localVariable2_copy = localVariable2;

				MatA[m][k] = 0;

				for(int n=k+1; n<3; n++)
				{
					#pragma HLS UNROLL
					MatA[m][n] -= MatA[k][n]*localVariable2;
				}

				for(int n=0; n<3; n++)
				{
					#pragma HLS UNROLL
					MatB[m][n] -= MatB[k][n]*localVariable2_copy;
				}
			}
	 	}
	}

    for(int m=0; m<3; m++)
    {
		for(int n=0; n<3; n++)
		{
			Minv[m][n] = MatB[m][n];
		}
    }

	return;
}

void Gaussian_Elimination_4d(float MatrixA[4][4], float MatrixB[4][4], float Minv[4][4])
{
	int j;
	float localVariable, localVariable_copy, localVariable2, localVariable2_copy;
	float MatA[4][4] = {0};
	float MatB[4][4] = {0};

	for(int m=0; m<4; m++)
	{
		for(int n=0; n<4; n++)
		{
			MatA[m][n] = MatrixA[m][n];
			MatB[m][n] = MatrixB[m][n];
		}
	}

	for(int k=0;k<4;k++)
	{
		// if diagonal is zero, then return if all row elements are 0
        if(MatA[k][k]==0)
        {
        	j = k + 1;
        	while(MatA[j][k] == 0)
        	{
                if(j >= 3)
                {
    				return;
                }
                else
                {
            		j = j + 1;
                }
			}

            for(int x=k; x < 4; x++)
            {
				#pragma HLS UNROLL
            	Swap(MatA[k][x], MatA[j][x]);
			}

            for(int x=0; x < 4; x++)
            {
				#pragma HLS UNROLL
            	Swap(MatB[k][x], MatB[j][x]);
			}
        }


        localVariable = 1/MatA[k][k];
		localVariable_copy = localVariable;
		MatA[k][k] = 1;

		for(int m=k+1; m<4; m++)
		{
			#pragma HLS UNROLL
			MatA[k][m]*=localVariable;
		}

		for(int m=0; m<4; m++)
		{
			#pragma HLS UNROLL
			MatB[k][m]*=localVariable_copy;
		}


	 	for(int m=0; m<4; m++)
	 	{
	 		if(m != k)
	 		{
				localVariable2 = MatA[m][k];
				localVariable2_copy = localVariable2;

				MatA[m][k] = 0;

				for(int n=k+1; n<4; n++)
				{
					#pragma HLS UNROLL
					MatA[m][n] -= MatA[k][n]*localVariable2;
				}

				for(int n=0; n<4; n++)
				{
					#pragma HLS UNROLL
					MatB[m][n] -= MatB[k][n]*localVariable2_copy;
				}
	 		}
	 	}
	}

    for(int m=0; m<4; m++)
    {
		for(int n=0; n<4; n++)
		{
			Minv[m][n] = MatB[m][n];
		}
    }

	return;
}


void Gaussian_Elimination_T_3d(float MatrixA[3][3], float MatrixB[3][3], float Minv[3][3])
{
	int j;
	float localVariable, localVariable_copy, localVariable2, localVariable2_copy;
	float MatA[3][3] = {0};
	float MatB[3][3] = {0};

	for(int m=0; m<3; m++)
	{
		for(int n=0; n<3; n++)
		{
			MatA[m][n] = MatrixA[m][n];
			MatB[m][n] = MatrixB[m][n];
		}
	}

	for(int k=0;k<3;k++)
	{
		// if diagonal is zero, then return if all row elements are 0
        if(MatA[k][k]==0)
        {
        	j = k + 1;
        	while(MatA[j][k] == 0)
        	{
                if(j >= 2)
                {
    				return;
                }
                else
                {
            		j = j + 1;
                }
			}

            for(int x=k; x < 3; x++)
            {
				#pragma HLS UNROLL
            	Swap(MatA[k][x], MatA[j][x]);
			}

            for(int x=0; x < 3; x++)
            {
				#pragma HLS UNROLL
            	Swap(MatB[k][x], MatB[j][x]);
			}
        }


        localVariable = 1/MatA[k][k];
		localVariable_copy = localVariable;
		MatA[k][k] = 1;

		for(int m=k+1; m<3; m++)
		{
			#pragma HLS UNROLL
			MatA[k][m]*=localVariable;
		}

		for(int m=0; m<3; m++)
		{
			#pragma HLS UNROLL
			MatB[k][m]*=localVariable_copy;
		}


	 	for(int m=0; m<3; m++)
	 	{
	 		if(m != k)
			{
				localVariable2 = MatA[m][k];
				localVariable2_copy = localVariable2;

				MatA[m][k] = 0;

				for(int n=k+1; n<3; n++)
				{
					#pragma HLS UNROLL
					MatA[m][n] -= MatA[k][n]*localVariable2;
				}

				for(int n=0; n<3; n++)
				{
					#pragma HLS UNROLL
					MatB[m][n] -= MatB[k][n]*localVariable2_copy;
				}
			}
	 	}
	}

    for(int m=0; m<3; m++)
    {
		for(int n=0; n<3; n++)
		{
			Minv[m][n] = MatB[n][m];
		}
    }

	return;
}

void Gaussian_Elimination_T_4d(float MatrixA[4][4], float MatrixB[4][4], float Minv[4][4])
{
	int j;
	float localVariable, localVariable_copy, localVariable2, localVariable2_copy;
	float MatA[4][4] = {0};
	float MatB[4][4] = {0};

	for(int m=0; m<4; m++)
	{
		for(int n=0; n<4; n++)
		{
			MatA[m][n] = MatrixA[m][n];
			MatB[m][n] = MatrixB[m][n];
		}
	}

	for(int k=0;k<4;k++)
	{
		// if diagonal is zero, then return if all row elements are 0
        if(MatA[k][k]==0)
        {
        	j = k + 1;
        	while(MatA[j][k] == 0)
        	{
                if(j >= 3)
                {
    				return;
                }
                else
                {
            		j = j + 1;
                }
			}

            for(int x=k; x < 4; x++)
            {
				#pragma HLS UNROLL
            	Swap(MatA[k][x], MatA[j][x]);
			}

            for(int x=0; x < 4; x++)
            {
				#pragma HLS UNROLL
            	Swap(MatB[k][x], MatB[j][x]);
			}
        }

        localVariable = 1/MatA[k][k];
		localVariable_copy = localVariable;
		MatA[k][k] = 1;

		for(int m=k+1; m<4; m++)
		{
			#pragma HLS UNROLL
			MatA[k][m]*=localVariable;
		}

		for(int m=0; m<4; m++)
		{
			#pragma HLS UNROLL
			MatB[k][m]*=localVariable_copy;
		}


	 	for(int m=0; m<4; m++)
	 	{
	 		if(m != k)
	 		{
				localVariable2 = MatA[m][k];
				localVariable2_copy = localVariable2;

				MatA[m][k] = 0;

				for(int n=k+1; n<4; n++)
				{
					#pragma HLS UNROLL
					MatA[m][n] -= MatA[k][n]*localVariable2;
				}

				for(int n=0; n<4; n++)
				{
					#pragma HLS UNROLL
					MatB[m][n] -= MatB[k][n]*localVariable2_copy;
				}
	 		}
	 	}
	}

	for(int m=0; m<4; m++)
	{
		for(int n=0; n<4; n++)
		{
			Minv[m][n] = MatB[n][m];
		}
	}

	return;
}


void Multiply_3d(float MatA[3][3], float MatB[3][3], float MatC[3][3])
{
//#pragma HLS ARRAY_PARTITION variable=MatB type=complete
	for (int i = 0; i < 3; i++) {
	    for (int k = 0; k < 3; k++) {
	    	float temp = 0;
	        for (int j = 0; j < 3; j++) {
	        	temp += MatA[i][j]*MatB[j][k];
	        }
	        MatC[i][k] = temp;
	    }
	}
	return;
}

void Multiply_4d(float MatA[4][4], float MatB[4][4], float MatC[4][4])
{
//#pragma HLS ARRAY_PARTITION variable=MatB type=complete
	for (int i = 0; i < 4; i++) {
	    for (int k = 0; k < 4; k++) {
	    	float temp = 0;
	        for (int j = 0; j < 4; j++) {
	        	temp += MatA[i][j]*MatB[j][k];
	        }
	        MatC[i][k] = temp;
	    }
	}
	return;
}

void Add_3d(float MatA[3][3], float MatB[3][3], float MatC[3][3])
{
#pragma HLS PIPELINE
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){
			MatC[i][j] = MatA[i][j] + MatB[i][j];
		}
	}
	return;
}

void Add_4d(float MatA[4][4], float MatB[4][4], float MatC[4][4])
{
#pragma HLS PIPELINE
	for (int i=0; i<4; i++){
		for (int j=0; j<4; j++){
			MatC[i][j] = MatA[i][j] + MatB[i][j];
		}
	}
	return;
}

void Transpose_3d(float MatA[3][3], float MatB[3][3])
{
#pragma HLS PIPELINE
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){
			MatB[j][i] = MatA[i][j];
		}
	}
	return;
}

void Transpose_4d(float MatA[4][4], float MatB[4][4])
{
#pragma HLS PIPELINE
	for (int i=0; i<4; i++){
		for (int j=0; j<4; j++){
			MatB[j][i] = MatA[i][j];
		}
	}
	return;
}

void Swap(float &A,float &B)
{
	float temp = A;
	A = B;
	B = temp;
	return;
}

void Saturate(float A,  float upper_limit, float lower_limit, float &B)
{
	B = (A>upper_limit)?upper_limit:(A<(lower_limit))?(lower_limit):A;
	return;
}
