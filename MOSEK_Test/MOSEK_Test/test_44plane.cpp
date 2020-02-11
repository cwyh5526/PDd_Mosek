//take two tetrahedron and test the 44 separating plane test
//calculate each value and take the minimum one
#include <stdio.h>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> //for matrices
#include <glm/gtc/type_ptr.hpp>

#include "mosek.h" /* Include the MOSEK definition file. */
#include <math.h>



using namespace std;
using namespace glm;

#define NUMCON 4   /* Number of constraints.             */
#define NUMVAR 12   /* Number of variables.               */
#define NUMANZ 12   /* Number of non-zeros in A.          */
#define NUMQNZ 30   /* Number of non-zeros in Q.          */


typedef struct{
	vec3 v[3];
	vec3 faceNormal;
}plane;
typedef struct{
	vec3 v[2];
}lineSeg;
typedef struct{
	vec3 vertex[4];
	plane face[4];
	lineSeg edge[6];
	vec3 faceConfstant[4];	
}tet;




/* This function prints log output from MOSEK to the terminal. */
static void MSKAPI printstr(void       *handle,
	const char str[])
{
	printf("%s", str);
} /* printstr */

void initTet(tet* T, vec3 v0, vec3 v1, vec3 v2, vec3 v3){
	T->vertex[0] = v0;
	T->vertex[1] = v1;
	T->vertex[2] = v2;
	T->vertex[3] = v3;

	T->face[0].v[0] = v1;	T->face[0].v[1] = v2;	T->face[0].v[2] = v3;
	T->face[1].v[0] = v0;	T->face[1].v[1] = v1;	T->face[1].v[2] = v3;
	T->face[2].v[0] = v0;	T->face[2].v[1] = v3;	T->face[2].v[2] = v2;
	T->face[3].v[0] = v0;	T->face[3].v[1] = v2;	T->face[3].v[2] = v1;
	
	T->edge[0].v[0] = v0;	T->edge[0].v[1] = v1;
	T->edge[1].v[0] = v0;	T->edge[1].v[1] = v2;
	T->edge[2].v[0] = v0;	T->edge[2].v[1] = v3;

	T->edge[3].v[0] = v1;	T->edge[3].v[1] = v2;
	T->edge[4].v[0] = v1;	T->edge[4].v[1] = v3;

	T->edge[5].v[0] = v2;	T->edge[5].v[1] = v3;
}

void init(tet *tetS, tet *tetR, tet *tetP, vec3 *rSum, float *rConstant){
	
	//tet으로 바꿔야하고
	//init에서 지금 sTet, rTet의 face와 edge를 계산해둬야하나

	//static tetrahedron position
	initTet(tetS,	vec3(0.0, 0.0, 0.0), 
					vec3(1.0, 0.0, 0.0), 
					vec3(0.0, 1.0, 0.0), 
					vec3(0.0, 0.0, 1.0));
	
	//rest pose tetrahedron position
	initTet(tetR,	vec3(0.2, 0.2, 0.2),
					vec3(1.2, 0.2, 0.2),
					vec3(0.2, 1.2, 0.2),
					vec3(0.7, 0.7, 1.2));	

	//deformed pose tetrahedron position same as the rest pose for the initial value
	initTet(tetP,	tetR->vertex[0],
					tetR->vertex[1],
					tetR->vertex[2],
					tetR->vertex[3]);
	
	//precompute the sum of coordinate values of rest pose tet. 
	(*rSum) = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < 4; i++){
		rSum->x += tetR->vertex[i].x;
		rSum->z += tetR->vertex[i].z;
		rSum->y += tetR->vertex[i].y;
	}

	//precompute constant term of the objective function
	vec3 constant = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < (i + 1); j++){
			constant.x += tetR->vertex[i].x * tetR->vertex[j].x;
			constant.y += tetR->vertex[i].y * tetR->vertex[j].y;
			constant.z += tetR->vertex[i].z * tetR->vertex[j].z;
		}
	}
	(*rConstant) = (float)(constant.x + constant.y + constant.z);

}


void opt(tet tetRest,vec3 normal, double constraintValue, vec3 rSum, float rConstant, double *metricValue, tet *tetDeform,double *time){

	//linear part of the obj function
	double c[] = { -(rSum.x + tetRest.vertex[0].x), -(rSum.x + tetRest.vertex[1].x), -(rSum.x + tetRest.vertex[2].x), -(rSum.x + tetRest.vertex[3].x),
		-(rSum.y + tetRest.vertex[0].y), -(rSum.y + tetRest.vertex[1].y), -(rSum.y + tetRest.vertex[2].y), -(rSum.y + tetRest.vertex[3].y),
		-(rSum.z + tetRest.vertex[0].z), -(rSum.z + tetRest.vertex[1].z), -(rSum.z + tetRest.vertex[2].z), -(rSum.z + tetRest.vertex[3].z) };


	/* Bounds on Constraints */
	// four constraints with low bound
	MSKboundkeye bkc[] = { MSK_BK_LO,
		MSK_BK_LO,
		MSK_BK_LO,
		MSK_BK_LO,
	};
	// four constraints with low bound value d
	double       blc[] = { constraintValue,
		constraintValue,
		constraintValue,
		constraintValue
	};
	// four constraints, upper bound value
	double       buc[] = { +MSK_INFINITY,
		+MSK_INFINITY,
		+MSK_INFINITY,
		+MSK_INFINITY
	};

	/* Bounds on Variables */
	// 12 variables with free bound
	MSKboundkeye bkx[] =/* {	MSK_BK_LO, MSK_BK_LO, MSK_BK_LO,
						MSK_BK_LO, MSK_BK_LO, MSK_BK_LO,
						MSK_BK_LO, MSK_BK_LO, MSK_BK_LO,
						MSK_BK_LO, MSK_BK_LO, MSK_BK_LO
						};*/
	{ MSK_BK_FR, MSK_BK_FR, MSK_BK_FR,
	MSK_BK_FR, MSK_BK_FR, MSK_BK_FR,
	MSK_BK_FR, MSK_BK_FR, MSK_BK_FR,
	MSK_BK_FR, MSK_BK_FR, MSK_BK_FR
	};
	// 12 variables  with free bound
	double       blx[] = /*{	0, 0, 0,
						 0, 0, 0,
						 0, 0, 0,
						 0, 0, 0
						 };*/

	{ -MSK_INFINITY, -MSK_INFINITY, -MSK_INFINITY,
	-MSK_INFINITY, -MSK_INFINITY, -MSK_INFINITY,
	-MSK_INFINITY, -MSK_INFINITY, -MSK_INFINITY,
	-MSK_INFINITY, -MSK_INFINITY, -MSK_INFINITY
	};
	// 12 variables  with free bound
	double       bux[] = { +MSK_INFINITY, +MSK_INFINITY, +MSK_INFINITY,
		+MSK_INFINITY, +MSK_INFINITY, +MSK_INFINITY,
		+MSK_INFINITY, +MSK_INFINITY, +MSK_INFINITY,
		+MSK_INFINITY, +MSK_INFINITY, +MSK_INFINITY
	};

	/* Below is the sparse representation of the A
	matrix stored by column. */
	MSKint32t	aptrb[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 },  //size: #of colum, begin index
		aptre[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 }, //size: #of colum, end index +1
		asub[] = { 0, //size: #of nonzeros,  array of row index in A that has nonzero values
		1,
		2,
		3,

		0,
		1,
		2,
		3,

		0,
		1,
		2,
		3
	};
	double      aval[] = { normal.x, //size: #of nonzeros,  array of nonzero values in A
		normal.x,
		normal.x,
		normal.x,

		normal.y,
		normal.y,
		normal.y,
		normal.y,

		normal.z,
		normal.z,
		normal.z,
		normal.z
	};



	MSKint32t     qsubi[NUMQNZ];
	MSKint32t     qsubj[NUMQNZ];
	double        qval[NUMQNZ];

	MSKint32t     i, j;
	double        xx[NUMVAR];

	MSKenv_t      env = NULL;
	MSKtask_t     task = NULL;
	MSKrescodee   r;


	/* Create the mosek environment. */
	r = MSK_makeenv(&env, NULL);

	if (r == MSK_RES_OK)
	{
		/* Create the optimization task. */
		r = MSK_maketask(env, NUMCON, NUMVAR, &task);

		if (r == MSK_RES_OK)
		{
			/* Directs the log task stream to the 'printstr' function. */
			r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);

			/* Append 'NUMCON' empty constraints.
			The constraints will initially have no bounds. */
			if (r == MSK_RES_OK)
				r = MSK_appendcons(task, NUMCON);

			/* Append 'NUMVAR' variables.
			The variables will initially be fixed at zero (x=0). */
			if (r == MSK_RES_OK)
				r = MSK_appendvars(task, NUMVAR);

			/* Optionally add a constant term to the objective. */
			if (r == MSK_RES_OK)
				r = MSK_putcfix(task, rConstant);

			for (j = 0; j < NUMVAR && r == MSK_RES_OK; ++j)
			{
				/* Set the linear term c_j in the objective.*/
				if (r == MSK_RES_OK)
					r = MSK_putcj(task, j, c[j]);

				/* Set the bounds on variable j.
				blx[j] <= x_j <= bux[j] */
				if (r == MSK_RES_OK)
					r = MSK_putvarbound(task,
					j,           /* Index of variable.*/
					bkx[j],      /* Bound key.*/
					blx[j],      /* Numerical value of lower bound.*/
					bux[j]);     /* Numerical value of upper bound.*/

				/* Input column j of A */
				if (r == MSK_RES_OK)
					r = MSK_putacol(task,
					j,                 /* Variable (column) index.*/
					aptre[j] - aptrb[j], /* Number of non-zeros in column j.*/
					asub + aptrb[j],   /* Pointer to row indexes of column j.*/
					aval + aptrb[j]);  /* Pointer to Values of column j.*/
			}

			/* Set the bounds on constraints.
			for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
			for (i = 0; i < NUMCON && r == MSK_RES_OK; ++i)
				r = MSK_putconbound(task,
				i,           /* Index of constraint.*/
				bkc[i],      /* Bound key.*/
				blc[i],      /* Numerical value of lower bound.*/
				buc[i]);     /* Numerical value of upper bound.*/

			if (r == MSK_RES_OK)
			{
				/*
				* The lower triangular part of the Q
				* matrix in the objective is specified.
				*/

				qsubi[0] = 0;   qsubj[0] = 0;  qval[0] = 2.0;

				qsubi[1] = 1;   qsubj[1] = 0;  qval[1] = 1.0;
				qsubi[2] = 1;   qsubj[2] = 1;  qval[2] = 2.0;

				qsubi[3] = 2;   qsubj[3] = 0;  qval[3] = 1.0;
				qsubi[4] = 2;   qsubj[4] = 1;  qval[4] = 1.0;
				qsubi[5] = 2;   qsubj[5] = 2;  qval[5] = 2.0;

				qsubi[6] = 3;   qsubj[6] = 0;  qval[6] = 1.0;
				qsubi[7] = 3;   qsubj[7] = 1;  qval[7] = 1.0;
				qsubi[8] = 3;   qsubj[8] = 2;  qval[8] = 1.0;
				qsubi[9] = 3;   qsubj[9] = 3;  qval[9] = 2.0;

				qsubi[10] = 4;   qsubj[10] = 4;  qval[10] = 2.0;

				qsubi[12] = 5;   qsubj[12] = 4;  qval[12] = 1.0;
				qsubi[11] = 5;   qsubj[11] = 5;  qval[11] = 2.0;

				qsubi[13] = 6;   qsubj[13] = 4;  qval[13] = 1.0;
				qsubi[14] = 6;   qsubj[14] = 5;  qval[14] = 1.0;
				qsubi[15] = 6;   qsubj[15] = 6;  qval[15] = 2.0;

				qsubi[16] = 7;   qsubj[16] = 4;  qval[16] = 1.0;
				qsubi[17] = 7;   qsubj[17] = 5;  qval[17] = 1.0;
				qsubi[18] = 7;   qsubj[18] = 6;  qval[18] = 1.0;
				qsubi[19] = 7;   qsubj[19] = 7;  qval[19] = 2.0;

				qsubi[20] = 8;   qsubj[20] = 8;  qval[20] = 2.0;

				qsubi[21] = 9;   qsubj[21] = 8;  qval[21] = 1.0;
				qsubi[22] = 9;   qsubj[22] = 9;  qval[22] = 2.0;

				qsubi[23] = 10;   qsubj[23] = 8;  qval[23] = 1.0;
				qsubi[24] = 10;   qsubj[24] = 9;  qval[24] = 1.0;
				qsubi[25] = 10;   qsubj[25] = 10;  qval[25] = 2.0;

				qsubi[26] = 11;   qsubj[26] = 8;  qval[26] = 1.0;
				qsubi[27] = 11;   qsubj[27] = 9;  qval[27] = 1.0;
				qsubi[28] = 11;   qsubj[28] = 10;  qval[28] = 1.0;
				qsubi[29] = 11;   qsubj[29] = 11;  qval[29] = 2.0;



				/* Input the Q for the objective. */

				r = MSK_putqobj(task, NUMQNZ, qsubi, qsubj, qval);
			}

			if (r == MSK_RES_OK)
			{
				MSKrescodee trmcode;

				/* Run optimizer */
				r = MSK_optimizetrm(task, &trmcode);

				/* Print a summary containing information
				about the solution for debugging purposes*/
				MSK_solutionsummary(task, MSK_STREAM_MSG);

				if (r == MSK_RES_OK)
				{
					MSKsolstae solsta;
					int j;

					MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

					switch (solsta)
					{
					case MSK_SOL_STA_OPTIMAL:
					case MSK_SOL_STA_NEAR_OPTIMAL:
						MSK_getxx(task,
							MSK_SOL_ITR,    /* Request the interior solution. */
							xx);

						printf("Optimal primal solution\n");
						for (j = 0; j < NUMVAR; ++j)
							printf("x[%d]: %e\n", j, xx[j]);

						break;

					case MSK_SOL_STA_DUAL_INFEAS_CER:
					case MSK_SOL_STA_PRIM_INFEAS_CER:
					case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
					case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
						printf("Primal or dual infeasibility certificate found.\n");
						break;

					case MSK_SOL_STA_UNKNOWN:
						printf("The status of the solution could not be determined.\n");
						break;

					default:
						printf("Other solution status.");
						break;
					}
				}
				else
				{
					printf("Error while optimizing.\n");
				}
			}

			if (r != MSK_RES_OK)
			{
				/* In case of an error print error code and description. */
				char symname[MSK_MAX_STR_LEN];
				char desc[MSK_MAX_STR_LEN];

				printf("An error occurred while optimizing.\n");
				MSK_getcodedesc(r,
					symname,
					desc);
				printf("Error %s - '%s'\n", symname, desc);
			}

			MSK_getprimalobj(task, MSK_SOL_ITR, metricValue);
			cout <<"metric Value"<< (*metricValue) << endl;
			tetDeform->vertex[0] = vec3(xx[0], xx[4], xx[8]);
			tetDeform->vertex[1] = vec3(xx[1], xx[5], xx[9]);
			tetDeform->vertex[2] = vec3(xx[2], xx[6], xx[10]);
			tetDeform->vertex[3] = vec3(xx[3], xx[7], xx[11]);

			double tm;
			MSK_getdouinf(task, MSK_DINF_OPTIMIZER_TIME, &tm);
			(*time) += tm;
		}
		MSK_deletetask(&task);
	}

	MSK_deleteenv(&env);




}

void separatingPlaneCalculation(vec3 edgeS[2],vec3 edgeR[2],  vec3 *normal, double *d){
	vec3 v01 = edgeS[1] - edgeS[0];
	vec3 v02 = edgeR[1] - edgeR[0];

	(*normal) = normalize(cross(v01, v02));

	*(d) = dot(*normal, edgeS[0]);
	std::cout << "normal: (" << (*normal).x << "," << (*normal).y << "," << (*normal).z << ")" << std::endl;
}

void separatingPlaneCalculation(vec3 face[3], vec3 *normal, double *d){
	vec3 v01 = face[1] - face[0];
	vec3 v02 = face[2] - face[0];

	(*normal) = normalize(cross(v01, v02));
	*(d) = dot(*normal, face[0]);
	std::cout << "normal: (" << (*normal).x << "," << (*normal).y << "," << (*normal).z << ")" << std::endl;

}
void separatingPlaneTest(tet tetStatic, tet tetRest, vec3 rSum, float rConstant, tet *tetDeform, double* minOptValue, int* minOptIndex){
	tet tetOpt[40];
	double metricValue[40] = { 100.0 };
	double minValue = 100.0;
	int minIndex=-1;

	vec3 normal;
	double constraintValue;

	double time=0.0;


	

	// 8 face tests
	for (int i = 0; i < 4; i++)
	{
		
		if (i < 4){
			separatingPlaneCalculation(tetStatic.face[i].v, &normal, &constraintValue);
			opt(tetRest, normal, constraintValue, rSum, rConstant, &(metricValue[i]), &(tetOpt[i]),&time);
		}
		else{
			//separatingPlaneCalculation(tetRest.face[i-4].v, &normal, &constraintValue);
			//opt(tetRest,normal, constraintValue, rSum, rConstant, metricValue, tetDeform);//Rest Tet은 자꾸 변할텐데?????이걸 Constraint로 줄 수 있는 것인가?
		}

		if (minValue>metricValue[i] && (metricValue[i] != 0)) { minValue = metricValue[i];	minIndex = i; }
	}
	
	// 36 edge tests
	//static edge 중의 한 점을 지나고 두 edge에 동시에 수직인 normal vector를 가지는 plane을 separating vector로.근데 그 방향이 바깥 방향이어야 하는데 어케 알징?
	//cross product가 counterclockwise여야 함.
	//그 벡터 방향으로 projection을 내렸을 때 길이를 비교해야 하는데, 
	/*for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			int index = 7 + i * 6 + j;
			separatingPlaneCalculation(tetStatic.edge[i].v, tetRest.edge[j].v, &normal, &constraintValue);
			opt(tetRest, normal, constraintValue, rSum, rConstant, &(metricValue[index]), &(tetOpt[index]),&time);

			if (minValue>metricValue[index] && (metricValue[i] != 0)){ minValue = metricValue[index]; 	minIndex = i; }
		}
	}
*/
	cout << "==Optimization result==" << endl;
	//minValue = 100.0;

	/*for (int i = 0; i < 40; i++){
		cout<<i<<"=" << metricValue[i] <<endl;
		if ((minValue>metricValue[i])&&(metricValue[i]!=0)){
			minValue = metricValue[i];
			minIndex = i;
		}
	}*/
	cout << "Total Opt Time =" <<  time<<endl;
	//take the minimum metric case
	if (minIndex != -1){
		(*tetDeform) = tetOpt[minIndex];
		(*minOptValue) = minValue;
		(*minOptIndex) = minIndex;
		std::cout << "tetDeform: p0= (" << (*tetDeform).vertex[0].x << "," << (*tetDeform).vertex[0].y << "," << (*tetDeform).vertex[0].z << ")" << std::endl;
		std::cout << "tetDeform: p1= (" << (*tetDeform).vertex[1].x << "," << (*tetDeform).vertex[1].y << "," << (*tetDeform).vertex[1].z << ")" << std::endl;
		std::cout << "tetDeform: p2= (" << (*tetDeform).vertex[2].x << "," << (*tetDeform).vertex[2].y << "," << (*tetDeform).vertex[2].z << ")" << std::endl;
		std::cout << "tetDeform: p3= (" << (*tetDeform).vertex[3].x << "," << (*tetDeform).vertex[3].y << "," << (*tetDeform).vertex[3].z << ")" << std::endl;
		cout << "Minimum Optimization Value: " << minValue << endl;
		printf("%f\n", minValue);
		cout << "Index of plane: " << minIndex << endl;

	}
	else printf("ERROR: Not Appropriate Tet Deform calculation ");
}

int main(int argc, const char *argv[])
{

	tet sTet; //static tetrahedron
	tet rTet; //rest pose tetrahedron
	tet pTet; //deformed pose tetrahedron

	vec3 rSum;		//the sum of each coordinate values of rest pose tet. 
	float rConstant;    //Cosntant term for the obj function

	double minOptValue;
	int minOptIndex;

	init(&sTet, &rTet, &pTet, &rSum, &rConstant);
	separatingPlaneTest(sTet, rTet, rSum,rConstant, &pTet, &minOptValue, &minOptIndex);


}