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

#ifndef __CER_MATRIX_H__
#define __CER_MATRIX_H__

#define SAFE_MODE

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define FOR_R(i) for (int i=0; i<R; ++i)
#define FOR_C(i) for (int i=0; i<C; ++i)
#define SCAN(r,c) FOR_R(r) FOR_C(c)

namespace cer {
namespace kinematics_alt {

class Matrix
{
public:
    Matrix(int r,int c=1,bool zero=true) : R(r),C(c)
    {
        allocate();

        if (zero) clear();
    }

    Matrix(const Matrix& M) : R(M.R),C(M.C)
    {
        allocate();
        SCAN(r,c) m[r][c]=M.m[r][c];
    }

    virtual ~Matrix()
    {
        if (m)
        {
            FOR_R(r) if (m[r]) delete [] m[r];
            delete [] m;
        }
    }

    void clear()
    {
        SCAN(r,c) m[r][c]=0.0;
    }

    double& operator()(int r,int c)
    {
        //if (r<0) r+=R;
        //if (c<0) c+=C;

        #ifdef SAFE_MODE
        if (r<0 || r>=R || c<0 || c>=C)
        {
            printf("Matrix:: index out of range ERROR\n");
            exit(-1);
        }
        #endif

        return m[r][c];
    }

    const double& operator()(int r,int c) const
    {
        //if (r<0) r+=R;
        //if (c<0) c+=C;

        #ifdef SAFE_MODE
        if (r<0 || r>=R || c<0 || c>=C)
        {
            printf("Matrix:: index out of range ERROR\n");
            exit(-1);
        }
        #endif

        return m[r][c];
    }

    double& operator()(int r)
    {
        #ifdef SAFE_MODE
        if (C!=1)
        {
            printf("Matrix:: single index dimension > 1 ERROR\n");
            exit(-1);
        }
        #endif
        //if (r<0) r+=R;
        return m[r][0];
    }

    Matrix t() const
    {
        Matrix trs(C,R,false);

        SCAN(r,c) trs.m[c][r]=m[r][c];

        return trs;
    }

    Matrix operator *(double x) const
    {
        Matrix mul(R,C,false);

        SCAN(r,c) mul.m[r][c]=x*m[r][c];

        return mul;
    }

    const Matrix& operator *=(double x)
    {
        SCAN(r,c) m[r][c]*=x;

        return *this;
    }

    Matrix operator/(double x) const
    {
        Matrix div(R,C,false);

        if (x==0.0)
        {
            printf("Matrix::/0.0 ERROR\n");
            exit(-1);
        }

        x=1.0/x;

        SCAN(r,c) div.m[r][c]=x*m[r][c];

        return div;
    }

    const Matrix& operator/=(double x)
    {
        if (x==0.0)
        {
            printf("Matrix::/=0.0 ERROR\n");
            exit(-1);
        }

        x=1.0/x;

        FOR_R(r) FOR_C(c) m[r][c]*=x;

        return *this;
    }

    Matrix operator *(const Matrix& M) const
    {
        #ifdef SAFE_MODE
        if (C!=M.R)
        {
            printf("Matrix::* incompatible dimension ERROR\n");
            exit(-1);
        }
        #endif

        Matrix mul(R,M.C);

        SCAN(r,t) for (int c=0; c<M.C; ++c) mul.m[r][c]+=m[r][t]*M.m[t][c];

        return mul;
    }

    const Matrix& operator=(const Matrix& M)
    {
        #ifdef SAFE_MODE
        if (R!=M.R || C!=M.C)
        {
            printf("Matrix::= incompatible dimension ERROR\n");
            printf("R1=%d  R2=%d\n",R,M.R);
            printf("C1=%d  C2=%d\n",C,M.C);
            exit(-1);
        }
        #endif

        SCAN(r,c) m[r][c]=M.m[r][c];

        return *this;
    }

    Matrix operator +(const Matrix& M) const
    {
        #ifdef SAFE_MODE
        if (R!=M.R || C!=M.C)
        {
            printf("Matrix::+ incompatible dimension ERROR\n");
            exit(-1);
        }
        #endif

        Matrix add(R,C,false);

        SCAN(r,c) add.m[r][c]=m[r][c]+M.m[r][c];

        return add;
    }

    const Matrix& operator +=(const Matrix& M)
    {
        #ifdef SAFE_MODE
        if (R!=M.R || C!=M.C)
        {
            printf("Matrix::+= incompatible dimension ERROR\n");
            exit(-1);
        }
        #endif

        SCAN(r,c) m[r][c]+=M.m[r][c];

        return *this;
    }

    Matrix operator -(const Matrix& M) const
    {
        #ifdef SAFE_MODE
        if (R!=M.R || C!=M.C)
        {
            printf("Matrix::- incompatible dimension ERROR\n");
            exit(-1);
        }
        #endif

        Matrix sub(R,C,false);

        SCAN(r,c) sub.m[r][c]=m[r][c]-M.m[r][c];

        return sub;
    }

    Matrix operator -() const
    {
        Matrix neg(R,C,false);

        SCAN(r,c) neg.m[r][c]=-m[r][c];

        return neg;
    }

    const Matrix& operator -=(const Matrix& M)
    {
        #ifdef SAFE_MODE
        if (R!=M.R || C!=M.C)
        {
            printf("Matrix::-= incompatible dimension ERROR\n");
            exit(-1);
        }
        #endif

        SCAN(r,c) m[r][c]-=M.m[r][c];

        return *this;
    }

    Matrix operator[](Matrix& B) const
    {
        return *this*B-B**this;
    }

    double det() const
    {
        double result=1.0;

        #ifdef SAFE_MODE
        if (R!=C)
        {
            printf("Matrix::inv() not squared ERROR\n");
            exit(-1);
        }
        #endif

        Matrix B=*this;

        for (int r=0; r<R-1; ++r)
        {
            int pivot=-1;
            double max2=0.0;
            for (int d=r; d<R; ++d)
            {
                double m2=B(d,r)*B(d,r);

                if (m2>max2)
                {
                    max2=m2;
                    pivot=d;
                }
            }

            if (pivot==-1)
            {
                return 0.0;
            }

            if (pivot!=r) // swap r and pivot rows
            {
                result=-result;

                FOR_C(c)
                {
                    double tb=B(r,c); B(r,c)=B(pivot,c); B(pivot,c)=tb;
                }
            }

            double P=-1.0/B(r,r);

            for (int rr=r+1; rr<R; ++rr)
            {
                double D=P*B(rr,r);

                FOR_C(c)
                {
                    B(rr,c)+=D*B(r,c);
                }
            }
        }

        FOR_R(r) result*=B(r,r);

        return result;
    }

    Matrix inv(double* W) const
    {
        Matrix WT=t();

        for (int i=0; i<WT.R; ++i) for (int j=0; j<WT.C; ++j) WT(i,j)*=W[i];

        //Matrix WT=fast_mul_diag_full(W,t());
        //Matrix WT=W*t();

        return WT*(*this*WT).inv();
    }

    Matrix inv() const
    {
        if (R!=C)
        {
            Matrix T=t();
            return T*(*this*T).inv();
            //printf("Matrix::inv() non squared matrix inversion ERROR\n");
            //exit(-1);
        }

        Matrix B=*this;
        Matrix I(C,C);
        FOR_C(i) I(i,i)=1.0;

        for (int r=0; r<R-1; ++r)
        {
            int pivot=-1;
            double max2=0.0;
            for (int d=r; d<R; ++d)
            {
                double m2=B(d,r)*B(d,r);

                if (m2>max2)
                {
                    max2=m2;
                    pivot=d;
                }
            }

            if (pivot==-1)
            {
                printf("Matrix::inv(%d,%d) singular inversion ERROR at row %d\n",R,C,r);
				dump();

                I.clear();
                return I;
            }
            else if (pivot!=r)
            {
                FOR_C(c)
                {
                    double tb=B(r,c); B(r,c)=B(pivot,c); B(pivot,c)=tb;
                    double ti=I(r,c); I(r,c)=I(pivot,c); I(pivot,c)=ti;
                }
            }

            double P=-1.0/B(r,r);

            for (int rr=r+1; rr<R; ++rr)
            {
                double D=P*B(rr,r);

                FOR_C(c)
                {
                    B(rr,c)+=D*B(r,c);
                    I(rr,c)+=D*I(r,c);
                }
            }
        }

        for (int r=R-1; r>0; --r)
        {
            double P=-1.0/B(r,r);

            for (int rr=r-1; rr>=0; --rr)
            {
                double D=P*B(rr,r);

                FOR_C(c)
                {
                    B(rr,c)+=D*B(r,c);
                    I(rr,c)+=D*I(r,c);
                }
            }
        }

        FOR_R(r)
        {
            double D=1.0/B(r,r);
            FOR_C(c) I(r,c)*=D;
        }

        return I;
    }

    static Matrix id(int n)
    {
        Matrix I(n,n);

        for (int i=0; i<n; ++i) I.m[i][i]=1.0;

        return I;
    }

    /*
    Matrix e(int n=12) const
    {
        #ifdef SAFE_MODE
        if (R!=C)
        {
            printf("Non squared matrix exponential\n");
            exit(-1);
        }
        #endif

        Matrix E=id(R);
        Matrix An=E;
        int nfatt=1;

        for (int i=1; i<=n; ++i)
        {
            E+=(An=An**this)/double(nfatt*=i);
        }

        return E;
    }
    */

    void dump(FILE* pfile=stdout) const
    {
        FOR_R(r)
        {
            FOR_C(c)
            {
                fprintf(pfile,"%.12lf   ",m[r][c]);
            }

            fprintf(pfile,"\n");
        }

        fprintf(pfile,"\n");
    }

    Matrix sub(int r0,int sizeR,int c0,int sizeC) const
    {
        Matrix s(sizeR,sizeC,false);

        for (int r=0; r<sizeR; ++r)
        {
            for (int c=0; c<sizeC; ++c)
            {
                s.m[r][c]=m[r+r0][c+c0];
            }
        }

        return s;
    }

    Matrix eigen2() const
    {
        double A=0.5*(m[0][0]+m[1][1]);
        double B=sqrt(A*A-m[0][0]*m[1][1]+m[0][1]*m[1][0]);

        Matrix eig(2);
        eig(0)=A+B;
        eig(1)=A-B;

        return eig;
    }

    void base2(Matrix& l,Matrix &B)
    {
        l=eigen2();

        if (m[1][0]!=0.0)
        {
            B(0,0)=l(0)-m[1][1];
            B(1,0)=     m[1][0];

            double D=1.0/sqrt(B(0,0)*B(0,0)+B(1,0)*B(1,0));

            B(0,0)*=D;
            B(1,0)*=D;

            B(0,1)=l(1)-m[1][1];
            B(1,1)=     m[1][0];

            D=1.0/sqrt(B(0,1)*B(0,1)+B(1,1)*B(1,1));

            B(0,1)*=D;
            B(1,1)*=D;
        }
        else if (m[0][1]!=0.0)
        {
            B(0,0)=     m[0][1];
            B(1,0)=l(0)-m[0][0];     

            double D=1.0/sqrt(B(0,0)*B(0,0)+B(1,0)*B(1,0));

            B(0,0)*=D;
            B(1,0)*=D;

            B(0,1)=     m[0][1];
            B(1,1)=l(1)-m[0][0];

            D=1.0/sqrt(B(0,1)*B(0,1)+B(1,1)*B(1,1));

            B(0,1)*=D;
            B(1,1)*=D;
        }
        else
        {
            B(0,0)=1.0; B(0,1)=0.0;
            B(1,0)=0.0; B(1,1)=1.0;
        }
    }

    Matrix eigen() const
    {
        #ifdef SAFE_MODE
        if (R!=3 || C!=3)
        {
            printf("Matrix::eigen() not 3x3 matrix ERROR\n");
            exit(-1);
        }
        #endif

        Matrix eig(3);

        double p1 = m[0][1]*m[0][1]+m[1][2]*m[1][2]+m[2][0]*m[2][0];

        if (p1==0.0) // M is diagonal
        {

            if (m[0][0]>=m[1][1] && m[0][0]>=m[2][2])
            {
                eig(0)=m[0][0];

                if (m[1][1]>=m[2][2])
                {
                    eig(1)=m[1][1];
                    eig(2)=m[2][2];
                }
                else
                {
                    eig(1)=m[2][2];
                    eig(2)=m[1][1];
                }
            }
            else if (m[1][1]>=m[2][2] && m[1][1]>=m[0][0])
            {
                eig(0)=m[1][1];

                if (m[2][2]>=m[0][0])
                {
                    eig(1)=m[2][2];
                    eig(2)=m[0][0];
                }
                else
                {
                    eig(1)=m[0][0];
                    eig(2)=m[2][2];
                }
            }
            else
            {
                eig(0)=m[2][2];

                if (m[0][0]>=m[1][1])
                {
                    eig(1)=m[0][0];
                    eig(2)=m[1][1];
                }
                else
                {
                    eig(1)=m[1][1];
                    eig(2)=m[0][0];
                }
            }
        }
        else
        {
            double t=m[0][0]+m[1][1]+m[2][2];
            double q=t/3.0;
            double q0=m[0][0]-q,q1=m[1][1]-q,q2=m[2][2]-q;
            double p2=q0*q0+q1*q1+q2*q2+2.0*p1;

            double p=sqrt(p2/6.0);

            Matrix B=(*this-id(3)*q)*(1/p);
            double r=B.det()/2.0;

            // In exact arithmetic for a symmetric matrix  -1 <= r <= 1
            // but computation error can leave it slightly outside this range.

            double phi;

            if (r<=-1.0)
            {
                phi=M_PI/3.0;
            }
            else if (r>=1.0)
            {
                phi=0.0;
            }
            else
            {
                phi=acos(r)/3.0;
            }

            // the eigenvalues satisfy eig3 <= eig2 <= eig1
            eig(0)=q+2.0*p*cos(phi);
            eig(2)=q+2.0*p*cos(phi+2.0*M_PI/3.0);
            eig(1)=t-eig(0)-eig(2); // since trace(M) = eig0 + eig1 + eig2
        }

        return eig;
    }

    void base(Matrix& l,Matrix &B)
    {
        l=eigen();

        double Ex[3],Ey[3],Ez[3];

        for (int d=0; d<2; ++d)
        {
            double Ux=m[1][2]*(l(d)-m[0][0])+m[2][0]*m[0][1];
            double Uy=m[2][0]*(l(d)-m[1][1])+m[0][1]*m[1][2];
            double Uz=m[0][1]*(l(d)-m[2][2])+m[1][2]*m[2][0];

            if (fabs(Ux)<=fabs(Uy) && fabs(Ux)<=fabs(Uz))
            {
                Ex[d]=1.0; Ey[d]=Ux/Uy; Ez[d]=Ux/Uz;
            }
            else if (fabs(Uy)<=fabs(Uz) && fabs(Uy)<=fabs(Ux))
            {
                Ey[d]=1.0; Ex[d]=Uy/Ux; Ez[d]=Uy/Uz;
            }
            else
            {
                Ez[d]=1.0; Ey[d]=Uz/Uy; Ex[d]=Uz/Ux;
            }

            double m=sqrt(Ex[d]*Ex[d]+Ey[d]*Ey[d]+Ez[d]*Ez[d]);

            if (m>0.0){ m=1.0/m; Ex[d]*=m; Ey[d]*=m; Ez[d]*=m; }
        }

        Ex[2]=Ey[0]*Ez[1]-Ey[1]*Ez[0];
        Ey[2]=Ez[0]*Ex[1]-Ez[1]*Ex[0];
        Ez[2]=Ex[0]*Ey[1]-Ex[1]*Ey[0];

        B(0,0)=Ex[0]; B(0,1)=Ex[1]; B(0,2)=Ex[2];
        B(1,0)=Ey[0]; B(1,1)=Ey[1]; B(1,2)=Ey[2];
        B(2,0)=Ez[0]; B(2,1)=Ez[1]; B(2,2)=Ez[2];
    }

    void Jacobi(Matrix& L,Matrix& B)
    {
        Matrix Li(R),Lj(R);
        Matrix Bi(R),Bj(R);

        for (int i=0; i<R; ++i) 
        {
            for (int j=0; j<R; ++j)
            {
                B(i,j)=(i==j);
                L(i,j)=m[i][j];
            }
        }

        int ip,jp;
        double absLij;

        double max;

        double Lii,Lij,Ljj;

        double teta,c,s,c2,sc,s2;

        for (int n=0; n<2*R; ++n)
        {
            max=0.0;

            for (int i=0; i<R-1; ++i)
            {
                for (int j=i+1; j<R; ++j)
                {
                    absLij=fabs(L(i,j));
                
                    if (absLij>max) { max=absLij; ip=i; jp=j; }
                }
            }

            Lii=L(ip,ip);
            Lij=L(ip,jp);
            Ljj=L(jp,jp);

            for (int k=0; k<R; ++k)
            {
                Li(k)=L(ip,k);
                Lj(k)=L(jp,k);
                
                Bi(k)=B(k,ip);
                Bj(k)=B(k,jp);
            }

            teta=0.5*atan2(2.0*Lij,Ljj-Lii);
            c=cos(teta);
            s=sin(teta);
            
            c2=c*c;
            sc=s*c;
            s2=s*s;

            L(ip,ip)=c2*Lii-2.0*sc*Lij+s2*Ljj;
            L(jp,jp)=s2*Lii+2.0*sc*Lij+c2*Ljj;
            L(ip,jp)=L(jp,ip)=(c2-s2)*Lij+sc*(Lii-Ljj);

            for (int k=0; k<R; ++k)
            {
                if (k!=ip && k!=jp)
                {
                    L(ip,k)=L(k,ip)=c*Li(k)-s*Lj(k);
                    L(jp,k)=L(k,jp)=s*Li(k)+c*Lj(k);
                }

                B(k,ip)=c*Bi(k)-s*Bj(k);
                B(k,jp)=s*Bi(k)+c*Bj(k);
            }
        }
    }

    const int R,C;

protected:
    //Matrix(int dim):R(dim),C(dim){ m=NULL; }

    void allocate()
    {
        m=new double*[R];

        FOR_R(r) m[r]=new double[C];
    }

    double abs(double x) const { return x>0.0?x:-x; }
    double sgn(double x) const { return x>=0.0?1.0:-1.0; }

    double** m;
};

inline Matrix operator *(double x,Matrix& M)
{
    return M*x;
}

inline Matrix fast_mul_diag_full(double *D,const Matrix& M)
{
    Matrix mul(M.R,M.C,false);

    for (int i=0; i<M.R; ++i)
    {
        for (int j=0; j<M.C; ++j)
        {
            mul(i,j)=D[i]*M(i,j);
        }
    }

    return mul;
}

/*
inline Matrix fast_mul_full_diag(const Matrix& M,const Matrix& D)
{
    #ifdef SAFE_MODE
    if (M.C!=D.R || D.C!=D.R)
    {
        printf("Matrix::mul_full_diag incompatible dimension ERROR\n");
        exit(-1);
    }
    #endif

    Matrix mul(M.R,M.C,false);

    for (int i=0; i<M.R; ++i)
    {
        for (int j=0; j<M.C; ++j)
        {
            mul(i,j)=M(i,j)*D(j,j);
        }
    }

    return mul;
}

inline Matrix fast_mul_diag_diag(const Matrix& D1,const Matrix& D2)
{
    #ifdef SAFE_MODE
    if (D1.R!=D1.C || D2.R!=D2.C || D1.R!=D2.R)
    {
        printf("Matrix::mul_diag_diag incompatible dimension ERROR\n");
        exit(-1);
    }
    #endif

    Matrix mul(D1.R,D1.R);

    for (int i=0; i<D1.R; ++i) mul(i,i)=D1(i,i)*D2(i,i);

    return mul;
}

inline Matrix fast_mul_diag_scalar(const Matrix& D,double x)
{
    #ifdef SAFE_MODE
    if (D.R!=D.C)
    {
        printf("Matrix::mul_diag_scalar incompatible dimension ERROR\n");
        exit(-1);
    }
    #endif

    Matrix mul(D.R,D.R);

    for (int i=0; i<D.R; ++i) mul(i,i)=D(i,i)*x;

    return mul;
}

inline Matrix fast_mul_scalar_diag(double x,const Matrix& D)
{
    return fast_mul_diag_scalar(D,x);
}
*/
}
}

#endif
