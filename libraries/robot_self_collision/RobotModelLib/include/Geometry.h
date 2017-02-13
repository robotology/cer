/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Alessandro Scalzo
 * email:  alessandro.scalzo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __CER_GEOMETRY_H__
#define __CER_GEOMETRY_H__

#include <math.h>
//#include <cmath>

#include "Matrix.h"

namespace cer
{
	namespace robot_model
	{

		const double DEG2RAD = M_PI / 180.0;
		const double RAD2DEG = 180.0 / M_PI;

		class Vec3
		{
		public:
			virtual ~Vec3(){}

			Vec3(){ z = y = x = 0.0; }
			Vec3(const Vec3& V) { x = V.x; y = V.y; z = V.z; }
			Vec3(double Vx, double Vy, double Vz){ x = Vx; y = Vy; z = Vz; }
			Vec3(double *V) { x = V[0]; y = V[1]; z = V[2]; }

			Vec3(Matrix &M)
			{
				if (M.R != 3 || M.C != 1)
				{
					printf("Vec3(Matrix &M) invalid dimension ERROR\n");
					exit(-1);
				}

				x = M(0); y = M(1); z = M(2);
			}

			operator Matrix()
			{
				Matrix M(3);
				M(0) = x;
				M(1) = y;
				M(2) = z;
				return M;
			}

			const Vec3& operator=(const Vec3& V)
			{
				x = V.x;
				y = V.y;
				z = V.z;
				return *this;
			}

			Vec3 operator+(const Vec3& V) const
			{
				return Vec3(x + V.x, y + V.y, z + V.z);
			}

			Vec3 operator-(const Vec3& V) const
			{
				return Vec3(x - V.x, y - V.y, z - V.z);
			}

			const Vec3& operator+=(const Vec3& V)
			{
				x += V.x;
				y += V.y;
				z += V.z;
				return *this;
			}

			const Vec3& operator-=(const Vec3& V)
			{
				x -= V.x;
				y -= V.y;
				z -= V.z;
				return *this;
			}

			Vec3 operator*(double a) const
			{
				return Vec3(x*a, y*a, z*a);
			}

			const Vec3& operator*=(double a)
			{
				x *= a; y *= a; z *= a;
				return *this;
			}

			Vec3 operator/(double a) const
			{
				if (a == 0.0)
				{
					printf("Vec3::operator/(0.0) ERROR\n");
					exit(-1);
				}

				a = 1.0 / a;

				return Vec3(x*a, y*a, z*a);
			}

			const Vec3& operator/=(double a)
			{
				if (a == 0.0)
				{
					printf("Vec3::operator/=(0.0) ERROR\n");
					exit(-1);
				}

				a = 1.0 / a;

				x *= a; y *= a; z *= a;

				return *this;
			}

			Vec3 operator-() const { return Vec3(-x, -y, -z); }

			double mod2() const { return x*x + y*y + z*z; }

			double mod() const { return sqrt(mod2()); }

			Vec3 norm(double u = 1.0) const
			{
				double m = mod();

				if (m > 0.0)
				{
					u /= m;
					return *this*u;
				}

				return *this;
			}

			double normalize(double u = 1.0)
			{
				double m = mod();

				if (m > 0.0)
				{
					u /= m;
					x *= u; y *= u; z *= u;
				}

				return m;
			}

			void clear(){ z = y = x = 0.0; }

			Matrix s() const
			{
				Matrix S(3, 3);

				S(0, 1) = -z; S(0, 2) = y;
				S(1, 0) = z;             S(1, 2) = -x;
				S(2, 0) = -y;  S(2, 1) = x;

				return S;
			}

			void print(char* head = "", char* tail = "") const
			{
				printf("%s x=%f y=%f z=%f %s\n", head, x, y, z, tail);
			}

			double x, y, z;

		protected:
			friend inline Vec3 operator%(const Vec3& U, const Vec3& V);
			friend inline double operator*(const Vec3& U, const Vec3& V);
		};

		inline Vec3 operator*(double x, const Vec3& V)
		{
			return V*x;
		}

		inline Vec3 operator%(const Vec3& U, const Vec3& V)
		{
			return Vec3(U.y*V.z - U.z*V.y, U.z*V.x - U.x*V.z, U.x*V.y - U.y*V.x);
		}

		inline double operator*(const Vec3& U, const Vec3& V)
		{
			return U.x*V.x + U.y*V.y + U.z*V.z;
		}

		class Quaternion
		{
		public:
			Quaternion() { s = 1.0; }
			Quaternion(Vec3& axis, double angle)
			{
				s = cos(angle *= 0.5*DEG2RAD);
				V = axis.norm(sin(angle));

				norm();
			}

			Quaternion(double qs, const Vec3& qV) :s(qs), V(qV)
			{
				norm();
			}

			virtual ~Quaternion(){}

			Quaternion conj() const { return Quaternion(s, -V); }

			Quaternion operator *(const Quaternion& Q) const
			{
				return Quaternion(s*Q.s - V*Q.V, s*Q.V + V*Q.s + V%Q.V).norm();
			}

			const Quaternion& norm()
			{
				if (s < 0.0)
				{
					s = -s;
					V = -V;
				}

				return *this;
			}

			double s;
			Vec3 V;
		};

#define FOR(i) for (int i=0; i<3; ++i)

		class Rotation
		{
		public:
			virtual ~Rotation(){}

			Rotation()
			{
				FOR(i) FOR(j) m[i][j] = double(i == j);
			}

			Rotation(const Rotation& R)
			{
				FOR(i) FOR(j) m[i][j] = R.m[i][j];
			}

			Rotation(double Rz)
			{
				Rz *= DEG2RAD;

				double cz = cos(Rz), sz = sin(Rz);

				m[0][0] = cz; m[0][1] = -sz; m[0][2] = 0.0;
				m[1][0] = sz; m[1][1] = cz; m[1][2] = 0.0;
				m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0;
			}

			Rotation(double Rx, double Ry, double Rz)
			{
				Rx *= DEG2RAD;
				Ry *= DEG2RAD;
				Rz *= DEG2RAD;

				double cx = cos(Rx), sx = sin(Rx);
				double cy = cos(Ry), sy = sin(Ry);
				double cz = cos(Rz), sz = sin(Rz);

				m[0][0] = cz*cy; m[0][1] = cz*sy*sx - sz*cx; m[0][2] = cz*sy*cx + sz*sx;
				m[1][0] = sz*cy; m[1][1] = sz*sy*sx + cz*cx; m[1][2] = sz*sy*cx - cz*sx;
				m[2][0] = -sy; m[2][1] = cy*sx;       m[2][2] = cy*cx;
			}

			Rotation(Vec3& rpy)
			{
				double Rx = DEG2RAD*rpy.x;
				double Ry = DEG2RAD*rpy.y;
				double Rz = DEG2RAD*rpy.z;

				double cx = cos(Rx), sx = sin(Rx);
				double cy = cos(Ry), sy = sin(Ry);
				double cz = cos(Rz), sz = sin(Rz);

				m[0][0] = cz*cy; m[0][1] = cz*sy*sx - sz*cx; m[0][2] = cz*sy*cx + sz*sx;
				m[1][0] = sz*cy; m[1][1] = sz*sy*sx + cz*cx; m[1][2] = sz*sy*cx - cz*sx;
				m[2][0] = -sy; m[2][1] = cy*sx;       m[2][2] = cy*cx;
			}

			Rotation(double theta, Vec3& u)
			{
				double ct = cos(theta);
				double st = sin(theta);
				double lct = 1.0 - ct;

				m[0][0] = ct + u.x*u.x*lct;     m[0][1] = u.x*u.y*lct - u.z*st; m[0][2] = u.x*u.z*lct + u.y*st;
				m[1][0] = u.x*u.y*lct + u.z*st; m[1][1] = ct + u.y*u.y*lct;     m[1][2] = u.y*u.z*lct - u.x*st;
				m[2][0] = u.x*u.z*lct - u.y*st; m[2][1] = u.y*u.z*lct + u.x*st; m[2][2] = ct + u.z*u.z*lct;
			}

			Rotation(Vec3& A, Vec3& B)
			{
				//A.normalize();
				//B.normalize();

				Vec3 u = A%B;

				double st = u.mod();
				double ct = A*B;
				double lct = 1.0 - ct;

				u.normalize();

				m[0][0] = ct + u.x*u.x*lct;     m[0][1] = u.x*u.y*lct - u.z*st; m[0][2] = u.x*u.z*lct + u.y*st;
				m[1][0] = u.x*u.y*lct + u.z*st; m[1][1] = ct + u.y*u.y*lct;     m[1][2] = u.y*u.z*lct - u.x*st;
				m[2][0] = u.x*u.z*lct - u.y*st; m[2][1] = u.y*u.z*lct + u.x*st; m[2][2] = ct + u.z*u.z*lct;
			}

			Rotation(Vec3 Ex, Vec3 Ey, Vec3 Ez)
			{
				Ex.normalize();
				Ey.normalize();
				Ez.normalize();

				m[0][0] = Ex.x; m[0][1] = Ey.x; m[0][2] = Ez.x;
				m[1][0] = Ex.y; m[1][1] = Ey.y; m[1][2] = Ez.y;
				m[2][0] = Ex.z; m[2][1] = Ey.z; m[2][2] = Ez.z;
			}

			Vec3 rpy() const
			{
				double Rz1 = atan2(m[1][0], m[0][0]);
				double Rx3 = atan2(m[2][1], m[2][2]);
				double Ry2 = atan2(-m[2][0], cos(Rz1)*m[0][0] + sin(Rz1)*m[1][0]);

				return RAD2DEG*Vec3(Rx3, Ry2, Rz1);
			}

			Vec3 eul() const
			{
				double Rz3 = atan2(m[2][0], m[2][1]);
				double Rz1 = atan2(m[0][2], -m[1][2]);
				double Rx2 = atan2(sin(Rz3)*m[2][0] + cos(Rz3)*m[2][1], m[2][2]);

				return RAD2DEG*Vec3(Rz3, Rx2, Rz1);
			}

			const Rotation& operator=(const Rotation& R)
			{
				FOR(i) FOR(j) m[i][j] = R.m[i][j];

				return *this;
			}

			Rotation inv() const
			{
				Rotation Rinv;

				FOR(i) FOR(j) Rinv.m[i][j] = m[j][i];

				return Rinv;
			}

			Rotation operator*(const Rotation& R) const
			{
				Rotation Rtot;

				FOR(r) FOR(c)
				{
					Rtot.m[r][c] = 0.0;

					FOR(i) Rtot.m[r][c] += m[r][i] * R.m[i][c];
				}

				return Rtot;
			}

			Rotation operator/(const Rotation& R) const
			{
				return *this*R.inv();
			}

			Vec3 Ex() const { return Vec3(m[0][0], m[1][0], m[2][0]); }
			Vec3 Ey() const { return Vec3(m[0][1], m[1][1], m[2][1]); }
			Vec3 Ez() const { return Vec3(m[0][2], m[1][2], m[2][2]); }

			Vec3 operator* (const Vec3& V) const
			{
				Vec3 W;

				W.x += m[0][0] * V.x + m[0][1] * V.y + m[0][2] * V.z;
				W.y += m[1][0] * V.x + m[1][1] * V.y + m[1][2] * V.z;
				W.z += m[2][0] * V.x + m[2][1] * V.y + m[2][2] * V.z;

				return W;
			}

			Quaternion quaternion() const
			{
				Quaternion Q;

				if (m[0][0] >= m[1][1] && m[0][0] >= m[2][2])
				{
					double r = 1.0 + m[0][0] - m[1][1] - m[2][2];

					if (r <= 0.0) return Q;

					r = sqrt(r);

					double s = 0.5 / r;
					Q.s = (m[2][1] - m[1][2])*s;
					Q.V.x = 0.5*r;
					Q.V.y = (m[0][1] + m[1][0])*s;
					Q.V.z = (m[2][0] + m[0][2])*s;
				}
				else if (m[1][1] >= m[0][0] && m[1][1] >= m[2][2])
				{
					double r = 1.0 - m[0][0] + m[1][1] - m[2][2];

					if (r <= 0.0) return Q;

					r = sqrt(r);

					double s = 0.5 / r;
					Q.s = (m[0][2] - m[2][0])*s;
					Q.V.x = (m[0][1] + m[1][0])*s;
					Q.V.y = 0.5*r;
					Q.V.z = (m[2][1] + m[1][2])*s;
				}
				else if (m[2][2] >= m[0][0] && m[2][2] >= m[1][1])
				{
					double r = 1.0 - m[0][0] - m[1][1] + m[2][2];

					if (r <= 0.0) return Q;

					r = sqrt(r);

					double s = 0.5 / r;
					Q.s = (m[1][0] - m[0][1])*s;
					Q.V.x = (m[2][0] + m[0][2])*s;
					Q.V.y = (m[2][1] + m[1][2])*s;
					Q.V.z = 0.5*r;
				}

				Q.norm();

				return Q;
			}

			Vec3 angleAxis() const
			{
				double cosTh = 0.5*(m[0][0] + m[1][1] + m[2][2] - 1.0);

				if (cosTh >= 1.0) return Vec3();

				if (cosTh <= -1.0)
				{
					Vec3 U;

					if (m[0][0] > -1.0) U.x = sqrt(0.5*(1.0 + m[0][0]));
					if (m[1][1] > -1.0) U.y = sqrt(0.5*(1.0 + m[1][1]));
					if (m[2][2] > -1.0) U.z = sqrt(0.5*(1.0 + m[2][2]));

					if (m[0][1] < 0.0) U.x = -U.x;
					if (m[1][2] < 0.0) U.z = -U.z;

					return M_PI*U.norm();
				}

				Vec3 U(m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]);

				return U.norm(acos(cosTh));
			}

			operator Matrix() const
			{
				Matrix M(3, 3);

				FOR(r) FOR(c) M(r, c) = m[r][c];

				return M;
			}

			double& operator()(int i, int j){ return m[i][j]; }

			void print(FILE* file) const
			{
				fprintf(file, "%+.9f %+.9f %+.9f\n", m[0][0], m[0][1], m[0][2]);
				fprintf(file, "%+.9f %+.9f %+.9f\n", m[1][0], m[1][1], m[1][2]);
				fprintf(file, "%+.9f %+.9f %+.9f\n", m[2][0], m[2][1], m[2][2]);
			}

		protected:
			double m[3][3];
		};

		class Transform
		{
		public:
			virtual ~Transform(){}

			Transform() : R(), P(){}

			Transform(double Rz) : R(Rz), P(){}

			Transform(double Dx, double Dz, double Rx, double Rz)
			{
				Rz *= DEG2RAD;
				Rx *= DEG2RAD;

				double cRz = cos(Rz), sRz = sin(Rz);
				double cRx = cos(Rx), sRx = sin(Rx);

				R(0, 0) = cRz;  R(0, 1) = -sRz*cRx; R(0, 2) = sRz*sRx; P.x = Dx*cRz;
				R(1, 0) = sRz;  R(1, 1) = cRz*cRx; R(1, 2) = -cRz*sRx; P.y = Dx*sRz;
				R(2, 0) = 0.0;  R(2, 1) = sRx; R(2, 2) = cRx; P.z = Dz;
			}

			Transform(double Rx, double Ry, double Rz, double Px, double Py, double Pz) : R(Rx, Ry, Rz), P(Px, Py, Pz){}

			Transform(const Transform& T) : R(T.R), P(T.P){}

			Transform(const Rotation& R0, const Vec3& P0) : R(R0), P(P0){}

			const Transform& operator=(const Transform& T)
			{
				R = T.R;
				P = T.P;

				return *this;
			}

			Transform operator*(const Transform& T) const { return Transform(R*T.R, R*T.P + P); }

			Vec3 operator*(const Vec3& V) const { return R*V + P; }

			Transform inv() const { return Transform(R.inv(), -(R.inv()*P)); }

			Vec3 Zj() const { return R.Ez(); }

			const Vec3& Pj() const { return P; }
			const Rotation& Rj() const { return R; }

			Vec3& Pj() { return P; }
			Rotation& Rj() { return R; }

			void dump(FILE* pfile = stdout)
			{
				fprintf(pfile, "%+.9f   %+.9f   %+.9f   %+.9f\n  ", R(0, 0), R(0, 1), R(0, 2), P.x);
				fprintf(pfile, "%+.9f   %+.9f   %+.9f   %+.9f\n  ", R(1, 0), R(1, 1), R(1, 2), P.y);
				fprintf(pfile, "%+.9f   %+.9f   %+.9f   %+.9f\n\n", R(2, 0), R(2, 1), R(2, 2), P.z);
			}

		protected:
			Rotation R;
			Vec3 P;
		};
	}
}

#endif
