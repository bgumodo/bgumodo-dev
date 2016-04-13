#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;
using namespace std;

void calc_Tn(const Eigen::VectorXd& Q, double elevator_z, Eigen::MatrixXd& Tn)
{
  double t2,t3,t4,t5,t6,t7,t12,t8,t9,t10,t11,t13,t14,t15,t16,t17,t18,t19,t20,t21,t22,t23,t24,t25,t26,t27,t28;
  double t29,t30,t31,t32,t33,t34;

  t2 = sin(Q(0));
  t3 = sin(Q(2));
  t4 = t2*t3;
  t5 = cos(Q(0));
  t6 = cos(Q(2));
  t7 = sin(Q(1));
  t12 = t5*t6*t7;
  t8 = t4-t12;
  t9 = cos(Q(3));
  t10 = cos(Q(1));
  t11 = sin(Q(3));
  t13 = cos(Q(4));
  t14 = t8*t11*9.999999999932537E-1;
  t15 = t5*t9*t10*9.999999999932537E-1;
  t16 = t5*t10*t11*3.673205103346574E-6;
  t17 = t14+t15+t16-t8*t9*3.673205103346574E-6;
  t18 = sin(Q(4));
  t19 = t2*t6;
  t20 = t3*t5*t7;
  t21 = t19+t20;
  t22 = t3*t5;
  t23 = t2*t6*t7;
  t24 = t22+t23;
  t25 = t9*t24*3.673205103346574E-6;
  t26 = t2*t9*t10*9.999999999932537E-1;
  t27 = t2*t10*t11*3.673205103346574E-6;
  t28 = t25+t26+t27-t11*t24*9.999999999932537E-1;
  t29 = t5*t6;
  t30 = t29-t2*t3*t7;
  t31 = t7*t9*9.999999999932537E-1;
  t32 = t7*t11*3.673205103346574E-6;
  t33 = t6*t10*t11*9.999999999932537E-1;
  t34 = t31+t32+t33-t6*t9*t10*3.673205103346574E-6;
  Tn(0,0) = t8*t9*(-9.999999999932537E-1)-t8*t11*3.673205103346574E-6-t5*t9*t10*3.673205103346574E-6+t5*t10*t11*9.999999999932537E-1;
  Tn(0,1) = -t13*t21-t17*t18;
  Tn(0,2) = -t13*t17+t18*t21;
  Tn(0,3) = t2*2.51156E-4-t2*t3*(2.8E1/1.25E2)+t5*t7*2.66305E-1-t8*t9*2.4475E-1+t8*t11*1.31003E-2+t5*t6*t7*(2.8E1/1.25E2)+t5*t9*t10*1.31003E-2+t5*t10*t11*2.4475E-1+1.02208E-1;
  Tn(1,0) = t9*t24*9.999999999932537E-1+t11*t24*3.673205103346574E-6-t2*t9*t10*3.673205103346574E-6+t2*t10*t11*9.999999999932537E-1;
  Tn(1,1) = t13*t30-t18*t28;
  Tn(1,2) = -t13*t28-t18*t30;
  Tn(1,3) = t5*(-2.51156E-4)+t3*t5*(2.8E1/1.25E2)+t2*t7*2.66305E-1+t9*t24*2.4475E-1-t11*t24*1.31003E-2+t2*t6*t7*(2.8E1/1.25E2)+t2*t9*t10*1.31003E-2+t2*t10*t11*2.4475E-1;
  Tn(2,0) = t7*t9*3.673205103346574E-6-t7*t11*9.999999999932537E-1+t6*t9*t10*9.999999999932537E-1+t6*t10*t11*3.673205103346574E-6;
  Tn(2.1) = t18*t34-t3*t10*t13;
  Tn(2.2) = t13*t34+t3*t10*t18;
  Tn(2,3) = elevator_z+t10*2.66305E-1+t6*t10*(2.8E1/1.25E2)-t7*t9*1.31003E-2-t7*t11*2.4475E-1+t6*t9*t10*2.4475E-1-t6*t10*t11*1.31003E-2+3.20661E-1;
  Tn(3,3) = 1.0;
}

void calc_FK(const Eigen::VectorXd& Q,const Eigen::VectorXd& Xd, double elevator_z, Eigen::VectorXd& FK)
{

	double t2;
	double t3;
	double t4;
	double t5;
	double t7;
	double t8;
	double t9;
	double t10;
	double t11;
	double t14;
	double t15;
	double t19;
	double t20;
	double t24;


	t2 = sin(Q(0));
	t3 = sin(Q(2));
	t4 = cos(Q(0));
	t5 = sin(Q(1));
	t7 = cos(Q(2));
	t8 = t2 * t3 - t4 * t5 * t7;
	t9 = cos(Q(3));
	t10 = cos(Q(1));
	t11 = sin(Q(3));
	t14 = t3 * t4 + t2 * t5 * t7;
	t15 = sin(Q(4));
	t19 = ((t5 * t9 * 0.99999999999325373 + t5 * t11 * 3.6732051033465739E-6) + t7
			* t10 * t11 * 0.99999999999325373) - t7 * t9 * t10 *
					3.6732051033465739E-6;
	t20 = cos(Q(4));
	t24 = t19 * t20 + t3 * t10 * t15;
	t15 = t15 * t19 - t3 * t10 * t20;
	FK(0) = ((((((((t2 * 0.000251156 - Xd(0)) - t2 * t3 * 0.224) + t4 * t5 *
			0.266305) - t8 * t9 * 0.24475) + t8 * t11 * 0.0131003) + t4 *
			t5 * t7 * 0.224) + t4 * t9 * t10 * 0.0131003) + t4 * t10 * t11 *
			0.24475) + 0.102208;
	FK(1) = (((((((t4 * -0.000251156 - Xd(1)) + t2 * t5 * 0.266305) + t3 * t4 *
			0.224) + t9 * t14 * 0.24475) - t11 * t14 * 0.0131003) + t2 * t5 *
			t7 * 0.224) + t2 * t9 * t10 * 0.0131003) + t2 * t10 * t11 * 0.24475;
	FK(2) = (((((((elevator_z + t10 * 0.266305) - Xd(2)) - t5 * t9 * 0.0131003)
			- t5 * t11 * 0.24475) + t7 * t10 * 0.224) + t7 * t9 * t10 *
			0.24475) - t7 * t10 * t11 * 0.0131003) + 0.320661;
	FK(3) = -Xd(3) + atan(t15 / t24);
	FK(4) = -Xd(4) - atan(1.0 / sqrt(t24 * t24 + t15 * t15) * (((t5 * t9 *
			3.6732051033465739E-6 - t5 * t11 * 0.99999999999325373) + t7 * t9 * t10 *
			0.99999999999325373) + t7 * t10 * t11 * 3.6732051033465739E-6));
}

void calc_r(const Eigen::VectorXd& FK,const Eigen::VectorXd& Xd,const Eigen::MatrixXd& Tn, Eigen::VectorXd& r)
{
	RowVectorXd n(3);
	RowVectorXd o(3);
	RowVectorXd a(3);
	VectorXd diff(3);

	diff = Xd - FK;

	n(0) = Tn(0,0);
	n(1) = Tn(1,0);
	n(2) = Tn(2,0);

	o(0) = Tn(0,1);
	o(1) = Tn(1,1);
	o(2) = Tn(2,1);

	a(0) = Tn(0,2);
	a(1) = Tn(1,2);
	a(2) = Tn(2,2);

	r(0) = n * diff;
	r(1) = o * diff;
	r(2) = a * diff;
	r(3) = 0.0;
	r(4) = 0.0;
	r(5) = 0.0;
}

void calc_J(const Eigen::VectorXd& Q,const Eigen::VectorXd& Xd, double elevator_z, Eigen::MatrixXd& J)
{
  double t2 = sin(Q(0));
  double t3 = sin(Q(2));
  double t4 = t2*t3;
  double t5 = cos(Q(0));
  double t6 = cos(Q(2));
  double t7 = sin(Q(1));
  double t15 = t5*t6*t7;
  double t8 = t4-t15;
  double t9 = cos(Q(3));
  double t10 = cos(Q(1));
  double t11 = sin(Q(3));
  double t12 = t3*t5;
  double t13 = t2*t6*t7;
  double t14 = t12+t13;
  double t16 = t8*t9*9.999999999932537E-1;
  double t17 = t8*t11*3.673205103346574E-6;
  double t18 = t5*t9*t10*3.673205103346574E-6;
  double t37 = t5*t10*t11*9.999999999932537E-1;
  double t19 = t16+t17+t18-t37;
  double t20 = t2*t7*2.66305E-1;
  double t21 = t9*t14*2.4475E-1;
  double t22 = t3*t5*(2.8E1/1.25E2);
  double t23 = t2*t9*t10*1.31003E-2;
  double t24 = t2*t6*t7*(2.8E1/1.25E2);
  double t25 = t2*t10*t11*2.4475E-1;
  double t26 = t9*t14*9.999999999932537E-1;
  double t27 = t11*t14*3.673205103346574E-6;
  double t28 = t2*t10*t11*9.999999999932537E-1;
  double t39 = t2*t9*t10*3.673205103346574E-6;
  double t29 = t26+t27+t28-t39;
  double t30 = t2*2.51156E-4;
  double t31 = t5*t7*2.66305E-1;
  double t32 = t8*t11*1.31003E-2;
  double t33 = t5*t9*t10*1.31003E-2;
  double t34 = t5*t6*t7*(2.8E1/1.25E2);
  double t35 = t5*t10*t11*2.4475E-1;
  double t51 = t2*t3*(2.8E1/1.25E2);
  double t52 = t8*t9*2.4475E-1;
  double t36 = -Xd(0)+t30+t31+t32+t33+t34+t35-t51-t52+3.02208E-1;
  double t56 = t5*2.51156E-4;
  double t57 = t11*t14*1.31003E-2;
  double t38 = -Xd(1)+t20+t21+t22+t23+t24+t25-t56-t57;
  double t40 = t10*2.66305E-1;
  double t41 = t6*t10*(2.8E1/1.25E2);
  double t42 = t6*t9*t10*2.4475E-1;
  double t58 = t7*t9*1.31003E-2;
  double t59 = t7*t11*2.4475E-1;
  double t60 = t6*t10*t11*1.31003E-2;
  double t43 = -Xd(2)+elevator_z+t40+t41+t42-t58-t59-t60+3.20661E-1;
  double t44 = t7*t9*3.673205103346574E-6;
  double t45 = t6*t9*t10*9.999999999932537E-1;
  double t46 = t6*t10*t11*3.673205103346574E-6;
  double t61 = t7*t11*9.999999999932537E-1;
  double t47 = t44+t45+t46-t61;
  double t48 = t2*t6;
  double t49 = t3*t5*t7;
  double t50 = t48+t49;
  double t53 = t5*t6;
  double t55 = t2*t3*t7;
  double t54 = t53-t55;
  double t62 = t8*t11*9.999999999932537E-1;
  double t63 = t5*t9*t10*9.999999999932537E-1;
  double t64 = t5*t10*t11*3.673205103346574E-6;
  double t67 = t8*t9*3.673205103346574E-6;
  double t65 = t62+t63+t64-t67;
  double t66 = sin(Q(4));
  double t68 = t65*t66;
  double t69 = cos(Q(4));
  double t70 = t50*t69;
  double t71 = t68+t70;
  double t72 = t9*t14*3.673205103346574E-6;
  double t73 = t2*t9*t10*9.999999999932537E-1;
  double t74 = t2*t10*t11*3.673205103346574E-6;
  double t76 = t11*t14*9.999999999932537E-1;
  double t75 = t72+t73+t74-t76;
  double t77 = t54*t69;
  double t78 = t2*t10*2.66305E-1;
  double t79 = t2*t6*t10*(2.8E1/1.25E2);
  double t80 = t2*t6*t9*t10*2.4475E-1;
  double t136 = t2*t7*t9*1.31003E-2;
  double t137 = t2*t7*t11*2.4475E-1;
  double t138 = t2*t6*t10*t11*1.31003E-2;
  double t81 = t78+t79+t80-t136-t137-t138;
  double t97 = t66*t75;
  double t82 = t77-t97;
  double t83 = t5*t10*2.66305E-1;
  double t84 = t5*t6*t10*(2.8E1/1.25E2);
  double t85 = t5*t6*t9*t10*2.4475E-1;
  double t147 = t5*t7*t9*1.31003E-2;
  double t148 = t5*t7*t11*2.4475E-1;
  double t149 = t5*t6*t10*t11*1.31003E-2;
  double t86 = t83+t84+t85-t147-t148-t149;
  double t87 = t7*t9*9.999999999932537E-1;
  double t88 = t7*t11*3.673205103346574E-6;
  double t89 = t6*t10*t11*9.999999999932537E-1;
  double t101 = t6*t9*t10*3.673205103346574E-6;
  double t90 = t87+t88+t89-t101;
  double t91 = t7*2.66305E-1;
  double t92 = t6*t7*(2.8E1/1.25E2);
  double t93 = t10*t11*2.4475E-1;
  double t94 = t9*t10*1.31003E-2;
  double t95 = t6*t7*t9*2.4475E-1;
  double t153 = t6*t7*t11*1.31003E-2;
  double t96 = t91+t92+t93+t94+t95-t153;
  double t98 = t9*t54*2.4475E-1;
  double t99 = t5*t6*(2.8E1/1.25E2);
  double t154 = t11*t54*1.31003E-2;
  double t155 = t2*t3*t7*(2.8E1/1.25E2);
  double t100 = t98+t99-t154-t155;
  double t102 = t66*t90;
  double t115 = t3*t10*t69;
  double t103 = t102-t115;
  double t104 = t3*t10*(2.8E1/1.25E2);
  double t105 = t3*t9*t10*2.4475E-1;
  double t160 = t3*t10*t11*1.31003E-2;
  double t106 = t104+t105-t160;
  double t107 = t9*t50*2.4475E-1;
  double t108 = t2*t6*(2.8E1/1.25E2);
  double t109 = t3*t5*t7*(2.8E1/1.25E2);
  double t163 = t11*t50*1.31003E-2;
  double t110 = t107+t108+t109-t163;
  double t111 = t8*t9*1.31003E-2;
  double t112 = t8*t11*2.4475E-1;
  double t113 = t5*t9*t10*2.4475E-1;
  double t164 = t5*t10*t11*1.31003E-2;
  double t114 = t111+t112+t113-t164;
  double t116 = t7*t9*2.4475E-1;
  double t117 = t6*t9*t10*1.31003E-2;
  double t118 = t6*t10*t11*2.4475E-1;
  double t165 = t7*t11*1.31003E-2;
  double t119 = t116+t117+t118-t165;
  double t120 = t9*t14*1.31003E-2;
  double t121 = t11*t14*2.4475E-1;
  double t122 = t2*t10*t11*1.31003E-2;
  double t166 = t2*t9*t10*2.4475E-1;
  double t123 = t120+t121+t122-t166;
  double t124 = t50*t66;
  double t130 = t65*t69;
  double t125 = t124-t130;
  double t126 = t69*t75;
  double t127 = t54*t66;
  double t128 = t126+t127;
  double t129 = t30+t31+t32+t33+t34+t35-t51-t52;
  double t131 = t20+t21+t22+t23+t24+t25-t56-t57;
  double t132 = t2*t7*t9*9.999999999932537E-1;
  double t133 = t2*t7*t11*3.673205103346574E-6;
  double t134 = t2*t6*t10*t11*9.999999999932537E-1;
  double t135 = t132+t133+t134-t2*t6*t9*t10*3.673205103346574E-6;
  double t139 = t10*t11*3.673205103346574E-6;
  double t140 = t9*t10*9.999999999932537E-1;
  double t141 = t6*t7*t9*3.673205103346574E-6;
  double t142 = t139+t140+t141-t6*t7*t11*9.999999999932537E-1;
  double t143 = t5*t7*t9*9.999999999932537E-1;
  double t144 = t5*t7*t11*3.673205103346574E-6;
  double t145 = t5*t6*t10*t11*9.999999999932537E-1;
  double t146 = t143+t144+t145-t5*t6*t9*t10*3.673205103346574E-6;
  double t150 = t69*t90;
  double t151 = t3*t10*t66;
  double t152 = t150+t151;
  double t156 = t9*t54*3.673205103346574E-6;
  double t157 = t156-t11*t54*9.999999999932537E-1;
  double t158 = t3*t9*t10*3.673205103346574E-6;
  double t159 = t158-t3*t10*t11*9.999999999932537E-1;
  double t161 = t9*t50*3.673205103346574E-6;
  double t162 = t161-t11*t50*9.999999999932537E-1;
  J(0,0) = -t19*(t5*(-2.51156E-4)+t20+t21+t22+t23+t24+t25-t11*t14*1.31003E-2)+t19*t38+t29*t36-t29*(t30+t31+t32+t33+t34+t35-t2*t3*(2.8E1/1.25E2)-t8*t9*2.4475E-1);
  J(0,1) = t43*(t9*t10*(-3.673205103346574E-6)+t10*t11*9.999999999932537E-1+t6*t7*t9*9.999999999932537E-1+t6*t7*t11*3.673205103346574E-6)+t19*t86-t29*t81+t47*t96-t38*(t2*t7*t9*3.673205103346574E-6-t2*t7*t11*9.999999999932537E-1+t2*t6*t9*t10*9.999999999932537E-1+t2*t6*t10*t11*3.673205103346574E-6)-t36*(t5*t7*t9*3.673205103346574E-6-t5*t7*t11*9.999999999932537E-1+t5*t6*t9*t10*9.999999999932537E-1+t5*t6*t10*t11*3.673205103346574E-6);
  J(0,2) = t43*(t3*t9*t10*9.999999999932537E-1+t3*t10*t11*3.673205103346574E-6)+t36*(t9*t50*9.999999999932537E-1+t11*t50*3.673205103346574E-6)-t38*(t9*t54*9.999999999932537E-1+t11*t54*3.673205103346574E-6)-t19*t110-t29*t100+t47*t106;
  J(0,3) = -t36*t65-t38*t75+t19*t114+t43*t90+t29*t123+t47*t119;
  J(0,4) = 0.0;
  J(1,0) = t38*t71+t36*t82-t71*t131-t82*t129;
  J(1,1) = -t43*(t66*t142+t3*t7*t69)+t71*t86-t81*t82+t96*t103-t38*(t66*t135-t2*t3*t10*t69)-t36*(t66*t146-t3*t5*t10*t69);
  J(1,2) = -t43*(t66*t159-t6*t10*t69)-t71*t110-t82*t100+t103*t106-t36*(t8*t69+t66*t162)+t38*(t14*t69+t66*t157);
  J(1,3) = t71*t114+t103*t119+t123*(t77-t97)+t19*t36*t66-t29*t38*t66-t43*t47*t66;
  J(1,4) = -t36*t125+t38*t128-t43*t152;
  J(2,0) = -t38*t125-t36*t128+t128*t129+t131*(t124-t130);
  J(2,1) = -t43*(t69*t142-t3*t7*t66)+t81*t128-t86*t125+t96*t152-t38*(t69*t135+t2*t3*t10*t66)-t36*(t69*t146+t3*t5*t10*t66);
  J(2,2) = -t43*(t69*t159+t6*t10*t66)+t100*t128+t106*t152+t110*(t124-t130)+t36*(t8*t66-t69*t162)-t38*(t14*t66-t69*t157);
  J(2,3) = -t114*t125-t123*t128+t119*t152+t19*t36*t69-t29*t38*t69-t43*t47*t69;
  J(2,4) = -t36*t71+t43*t103+t38*(t77-t97);
  J(3,0) = 0.0;
  J(3,1) = 0.0;
  J(3,2) = 0.0;
  J(3,3) = 0.0;
  J(3,4) = 0.0;
  J(4,0) = 0.0;
  J(4,1) = 0.0;
  J(4,2) = 0.0;
  J(4,3) = 0.0;
  J(4,4) = 0.0;
  J(5,0) = 0.0;
  J(5,1) = 0.0;
  J(5,2) = 0.0;
  J(5,3) = 0.0;
  J(5,4) = 0.0;
}

void calc_Jinv(const Eigen::MatrixXd& J, Eigen::MatrixXd& Jinv)
{
	MatrixXd temp(5,5);

	temp = J.transpose() * J;

	Jinv = temp.inverse() * J.transpose();
}

void calc_g(const Eigen::VectorXd& Q,const Eigen::VectorXd& Xd, double elevator_z, Eigen::VectorXd& g)
{
  double t2 = cos(Q(0));
  double t3 = sin(Q(2));
  double t4 = t2*t3;
  double t5 = cos(Q(2));
  double t6 = sin(Q(0));
  double t7 = sin(Q(1));
  double t8 = t5*t6*t7;
  double t9 = t4+t8;
  double t10 = cos(Q(3));
  double t11 = cos(Q(1));
  double t12 = sin(Q(3));
  double t13 = t3*t6;
  double t15 = t2*t5*t7;
  double t14 = t13-t15;
  double t16 = t10*t14*9.999999999932537E-1;
  double t17 = t12*t14*3.673205103346574E-6;
  double t18 = t2*t10*t11*3.673205103346574E-6;
  double t27 = t2*t11*t12*9.999999999932537E-1;
  double t19 = t16+t17+t18-t27;
  double t20 = t6*t7*2.66305E-1;
  double t21 = t9*t10*2.4475E-1;
  double t22 = t2*t3*(2.8E1/1.25E2);
  double t23 = t6*t10*t11*1.31003E-2;
  double t24 = t5*t6*t7*(2.8E1/1.25E2);
  double t25 = t6*t11*t12*2.4475E-1;
  double t28 = t2*2.51156E-4;
  double t29 = t9*t12*1.31003E-2;
  double t26 = -Xd(1)+t20+t21+t22+t23+t24+t25-t28-t29;
  double t30 = t9*t10*9.999999999932537E-1;
  double t31 = t9*t12*3.673205103346574E-6;
  double t32 = t6*t11*t12*9.999999999932537E-1;
  double t40 = t6*t10*t11*3.673205103346574E-6;
  double t33 = t30+t31+t32-t40;
  double t34 = t6*2.51156E-4;
  double t35 = t2*t7*2.66305E-1;
  double t36 = t12*t14*1.31003E-2;
  double t37 = t2*t10*t11*1.31003E-2;
  double t38 = t2*t5*t7*(2.8E1/1.25E2);
  double t39 = t2*t11*t12*2.4475E-1;
  double t42 = t3*t6*(2.8E1/1.25E2);
  double t43 = t10*t14*2.4475E-1;
  double t41 = -Xd(0)+t34+t35+t36+t37+t38+t39-t42-t43+3.02208E-1;
  double t44 = t11*2.66305E-1;
  double t45 = t5*t11*(2.8E1/1.25E2);
  double t46 = t5*t10*t11*2.4475E-1;
  double t78 = t7*t10*1.31003E-2;
  double t79 = t7*t12*2.4475E-1;
  double t80 = t5*t11*t12*1.31003E-2;
  double t47 = -Xd(2)+elevator_z+t44+t45+t46-t78-t79-t80+3.20661E-1;
  double t48 = cos(Q(4));
  double t49 = sin(Q(4));
  double t50 = t12*t14*9.999999999932537E-1;
  double t51 = t2*t10*t11*9.999999999932537E-1;
  double t52 = t2*t11*t12*3.673205103346574E-6;
  double t68 = t10*t14*3.673205103346574E-6;
  double t53 = t50+t51+t52-t68;
  double t54 = t5*t6;
  double t55 = t2*t3*t7;
  double t56 = t54+t55;
  double t69 = t48*t53;
  double t70 = t49*t56;
  double t57 = t69-t70;
  double t58 = t9*t10*3.673205103346574E-6;
  double t59 = t6*t10*t11*9.999999999932537E-1;
  double t60 = t6*t11*t12*3.673205103346574E-6;
  double t72 = t9*t12*9.999999999932537E-1;
  double t61 = t58+t59+t60-t72;
  double t62 = t48*t61;
  double t63 = t2*t5;
  double t73 = t3*t6*t7;
  double t64 = t63-t73;
  double t65 = t49*t64;
  double t66 = t62+t65;
  double t67 = t34+t35+t36+t37+t38+t39-t42-t43;
  double t71 = t20+t21+t22+t23+t24+t25-t28-t29;
  double t74 = t7*t10*9.999999999932537E-1;
  double t75 = t7*t12*3.673205103346574E-6;
  double t76 = t5*t11*t12*9.999999999932537E-1;
  double t95 = t5*t10*t11*3.673205103346574E-6;
  double t77 = t74+t75+t76-t95;
  double t81 = t49*t53;
  double t82 = t48*t56;
  double t83 = t81+t82;
  double t84 = t49*t61;
  double t86 = t48*t64;
  double t85 = t84-t86;
  double t87 = t26*t33;
  double t88 = t7*t10*3.673205103346574E-6;
  double t89 = t5*t10*t11*9.999999999932537E-1;
  double t90 = t5*t11*t12*3.673205103346574E-6;
  double t94 = t7*t12*9.999999999932537E-1;
  double t91 = t88+t89+t90-t94;
  double t92 = t47*t91;
  double t141 = t19*t41;
  double t93 = t87+t92-t141;
  double t96 = t48*t77;
  double t97 = t3*t11*t49;
  double t98 = t96+t97;
  double t99 = t26*t66;
  double t100 = t41*t57;
  double t142 = t47*t98;
  double t101 = t99+t100-t142;
  double t102 = t6*t11*2.66305E-1;
  double t103 = t5*t6*t11*(2.8E1/1.25E2);
  double t104 = t5*t6*t10*t11*2.4475E-1;
  double t125 = t6*t7*t10*1.31003E-2;
  double t126 = t6*t7*t12*2.4475E-1;
  double t127 = t5*t6*t11*t12*1.31003E-2;
  double t105 = t102+t103+t104-t125-t126-t127;
  double t106 = t2*t11*2.66305E-1;
  double t107 = t2*t5*t11*(2.8E1/1.25E2);
  double t108 = t2*t5*t10*t11*2.4475E-1;
  double t136 = t2*t7*t10*1.31003E-2;
  double t137 = t2*t7*t12*2.4475E-1;
  double t138 = t2*t5*t11*t12*1.31003E-2;
  double t109 = t106+t107+t108-t136-t137-t138;
  double t110 = t7*2.66305E-1;
  double t111 = t5*t7*(2.8E1/1.25E2);
  double t112 = t11*t12*2.4475E-1;
  double t113 = t10*t11*1.31003E-2;
  double t114 = t5*t7*t10*2.4475E-1;
  double t140 = t5*t7*t12*1.31003E-2;
  double t115 = t110+t111+t112+t113+t114-t140;
  double t116 = t49*t77;
  double t139 = t3*t11*t48;
  double t117 = t116-t139;
  double t118 = t26*t85;
  double t119 = t41*t83;
  double t153 = t47*t117;
  double t120 = t118+t119-t153;
  double t121 = t6*t7*t10*9.999999999932537E-1;
  double t122 = t6*t7*t12*3.673205103346574E-6;
  double t123 = t5*t6*t11*t12*9.999999999932537E-1;
  double t124 = t121+t122+t123-t5*t6*t10*t11*3.673205103346574E-6;
  double t128 = t11*t12*3.673205103346574E-6;
  double t129 = t10*t11*9.999999999932537E-1;
  double t130 = t5*t7*t10*3.673205103346574E-6;
  double t131 = t128+t129+t130-t5*t7*t12*9.999999999932537E-1;
  double t132 = t2*t7*t10*9.999999999932537E-1;
  double t133 = t2*t7*t12*3.673205103346574E-6;
  double t134 = t2*t5*t11*t12*9.999999999932537E-1;
  double t135 = t132+t133+t134-t2*t5*t10*t11*3.673205103346574E-6;
  double t143 = t10*t64*2.4475E-1;
  double t144 = t2*t5*(2.8E1/1.25E2);
  double t154 = t12*t64*1.31003E-2;
  double t155 = t3*t6*t7*(2.8E1/1.25E2);
  double t145 = t143+t144-t154-t155;
  double t146 = t3*t11*(2.8E1/1.25E2);
  double t147 = t3*t10*t11*2.4475E-1;
  double t160 = t3*t11*t12*1.31003E-2;
  double t148 = t146+t147-t160;
  double t149 = t10*t56*2.4475E-1;
  double t150 = t5*t6*(2.8E1/1.25E2);
  double t151 = t2*t3*t7*(2.8E1/1.25E2);
  double t163 = t12*t56*1.31003E-2;
  double t152 = t149+t150+t151-t163;
  double t156 = t10*t64*3.673205103346574E-6;
  double t157 = t156-t12*t64*9.999999999932537E-1;
  double t158 = t3*t10*t11*3.673205103346574E-6;
  double t159 = t158-t3*t11*t12*9.999999999932537E-1;
  double t161 = t10*t56*3.673205103346574E-6;
  double t162 = t161-t12*t56*9.999999999932537E-1;
  double t164 = t10*t14*1.31003E-2;
  double t165 = t12*t14*2.4475E-1;
  double t166 = t2*t10*t11*2.4475E-1;
  double t176 = t2*t11*t12*1.31003E-2;
  double t167 = t164+t165+t166-t176;
  double t168 = t7*t10*2.4475E-1;
  double t169 = t5*t10*t11*1.31003E-2;
  double t170 = t5*t11*t12*2.4475E-1;
  double t177 = t7*t12*1.31003E-2;
  double t171 = t168+t169+t170-t177;
  double t172 = t9*t10*1.31003E-2;
  double t173 = t9*t12*2.4475E-1;
  double t174 = t6*t11*t12*1.31003E-2;
  double t178 = t6*t10*t11*2.4475E-1;
  double t175 = t172+t173+t174-t178;
  g(0) = t93*(t19*t26+t33*t41-t19*t71-t33*t67)*-2.0+t101*(t26*t57-t41*t66-t57*t71+t66*t67)*2.0+t120*(t26*t83-t41*t85+t67*t85-t71*t83)*2.0;
  g(1) = t93*(t47*(t10*t11*(-3.673205103346574E-6)+t11*t12*9.999999999932537E-1+t5*t7*t10*9.999999999932537E-1+t5*t7*t12*3.673205103346574E-6)+t19*t109-t33*t105+t91*t115-t41*(t2*t7*t10*3.673205103346574E-6-t2*t7*t12*9.999999999932537E-1+t2*t5*t10*t11*9.999999999932537E-1+t2*t5*t11*t12*3.673205103346574E-6)-t26*(t6*t7*t10*3.673205103346574E-6-t6*t7*t12*9.999999999932537E-1+t5*t6*t10*t11*9.999999999932537E-1+t5*t6*t11*t12*3.673205103346574E-6))*-2.0-t101*(t47*(t48*t131-t3*t7*t49)-t57*t109-t66*t105-t98*t115+t26*(t48*t124+t3*t6*t11*t49)+t41*(t48*t135+t2*t3*t11*t49))*2.0-t120*(t47*(t49*t131+t3*t7*t48)-t85*t105-t83*t109-t115*t117+t26*(t49*t124-t3*t6*t11*t48)+t41*(t49*t135-t2*t3*t11*t48))*2.0;
  g(2) = t93*(t47*(t3*t10*t11*9.999999999932537E-1+t3*t11*t12*3.673205103346574E-6)+t41*(t10*t56*9.999999999932537E-1+t12*t56*3.673205103346574E-6)-t26*(t10*t64*9.999999999932537E-1+t12*t64*3.673205103346574E-6)-t19*t152-t33*t145+t91*t148)*-2.0-t101*(t47*(t48*t159+t5*t11*t49)+t57*t152-t66*t145-t98*t148+t26*(t9*t49-t48*t157)-t41*(t14*t49-t48*t162))*2.0-t120*(t47*(t49*t159-t5*t11*t48)-t85*t145+t83*t152-t117*t148-t26*(t9*t48+t49*t157)+t41*(t14*t48+t49*t162))*2.0;
  g(3) = t101*(t57*t167-t66*t175+t98*t171-t26*t33*t48+t19*t41*t48-t47*t48*t91)*2.0+t120*(t83*t167-t85*t175+t117*t171-t26*t33*t49+t19*t41*t49-t47*t49*t91)*2.0-t93*(-t26*t61-t41*t53+t47*t77+t19*t167+t33*t175+t91*t171)*2.0;
  g(4) = 0.0;
}

void calc_Qinit(const Eigen::VectorXd& Xd,double elevator_z, Eigen::VectorXd& Qinit)
{
	Qinit(0) = atan2(Xd(1),Xd(0)); 						//theta 1
	Qinit(1) = acos((Xd(2)-elevator_z-0.3075)/0.4903); 	//theta 2
	Qinit(2) = 0.0;										//theta 3
	Qinit(3) = M_PI/2 - Qinit(1); 						//theta 4
	Qinit(4) = 0.0; 									//theta 5
}

std::vector<double> IK(geometry_msgs::Pose desired_pose)
{
	MatrixXd 		Tn(4,4);
	VectorXd 		FK(5), g(5), Q(5),Qinit(5), delta_NR(5), delta(5), reduce(5);
	VectorXd 		r(6), Xd(6), temp(6);
	MatrixXd 		J(6,5);
	MatrixXd 		Jinv(5,6);
	double 			zeta, G, miu, Delta, lamda;
	int 			n_iter       						= 0;
	bool 			flag;
	const double 	EPSILON 	  						= 1E-8;
	const double 	DSTEP 		   						= 0.00001;
	const double 	DMAX 		     					= 20.0;
	const double 	BETA		     					= 0.5;
	const double 	elevator_z 	 						= 0.4;

	double 			joint_limits[5][2];

	joint_limits[0][0] = -M_PI/2;
	joint_limits[0][1] = M_PI/2;
	joint_limits[1][0] = 0.0;
	joint_limits[1][1] = 1.69296;
	joint_limits[2][0] = -2.4434;
	joint_limits[2][1] = 2.4434;
	joint_limits[3][0] = -2.4434;
	joint_limits[3][1] = 2.4434;
	joint_limits[4][0] = -1.91986;
	joint_limits[4][1] = 1.91986;


	Xd << 	desired_pose.position.x,
			desired_pose.position.y,
			desired_pose.position.z,
			0.0,
			0.0,
			0.0;

	calc_Qinit(Xd,elevator_z,Qinit);
	Q = Qinit;
	zeta = Q.norm();
	bool solution = false;
	calc_Tn(Q,elevator_z,Tn);
	calc_FK(Q,Xd,elevator_z,FK);
	calc_r(FK,Xd,Tn,r);
	G = r.transpose() * r;
	while (!solution)
	{
		n_iter++;
		ROS_INFO("Algorithm Step 1");
		calc_J(Q,Xd,elevator_z,J);
		calc_Jinv(J,Jinv);
		ROS_INFO("Algorithm Step 2");
		delta_NR = -Jinv * r;
		ROS_INFO("Algorithm Step 3");
		calc_g(Q,Xd,elevator_z,g);
		ROS_INFO("Algorithm Step 4");
		if (G >= zeta*g.norm())
		{
			ROS_ERROR("Convergence to a local minimum. terminate the process!");
			break;
		}
		temp = J * g;
		miu = g.norm() / pow(temp.norm(),2);
		Delta = fmax(DSTEP, fmin(DMAX,miu*g.norm()));
		ROS_INFO("Algorithm Step 5");
		do{
			if (miu*g.norm() >= Delta)
			{
				if (delta_NR.norm() <= Delta)
				{
					delta = delta_NR;
				}
				else
				{
					delta = Delta*g / g.norm();
				}
			}
			else
			{
				lamda = 0.0;
				do
				{
					lamda += 0.01;
					reduce = (1-lamda)*miu*g + lamda*delta_NR;
				} while((reduce.norm() - Delta > 1E-3) && lamda <= 1.0);
				delta = (1-lamda)*miu*g + lamda*delta_NR;
			}
			ROS_INFO("Algorithm Step 6");
			flag = true;
			for (int i=0; i<5; i++) 
			{
				if (delta(i) < joint_limits[i][0] || delta(i) > joint_limits[i][1])
				{
					flag 	= false;
					Delta 	= max(BETA*Delta,DSTEP); 
					break;
				}
			}
		}while(!flag);
		Q 	= Q + delta;
		calc_Tn(Q,elevator_z,Tn);
		calc_FK(Q,Xd,elevator_z,FK);
		calc_r(FK,Xd,Tn,r);
		G = r.transpose() * r;
		if (G < EPSILON)
			solution = true;
	}

	
	std::vector<double> group_variable_values;
	group_variable_values.resize(5);
	group_variable_values[0] = Qinit(0);
	group_variable_values[1] = Qinit(1);
	group_variable_values[2] = Qinit(2);
	group_variable_values[3] = Qinit(3);
	group_variable_values[4] = Qinit(4);
	if(solution)
	{
		cout << Q << endl;
		ROS_INFO("Number of iterations: %d",n_iter);
		group_variable_values[0] = Q(0);
		group_variable_values[1] = Q(1);
		group_variable_values[2] = Q(2);
		group_variable_values[3] = Q(3);
		group_variable_values[4] = Q(4);
	}

	return group_variable_values;
}
