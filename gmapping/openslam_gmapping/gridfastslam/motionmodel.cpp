#include <gmapping/gridfastslam/motionmodel.h>
#include <gmapping/utils/stat.h>
#include <iostream>

#define MotionModelConditioningLinearCovariance 0.01
#define MotionModelConditioningAngularCovariance 0.001

namespace GMapping {



/*
@desc  里程计运动模型
@param 车子当前位置
@param 车子的线性位移
@param 车子的角度位移
*/
OrientedPoint 
MotionModel::drawFromMotion (const OrientedPoint& p, double linearMove, double angularMove) const{
	OrientedPoint n(p);
	double lm=linearMove  + fabs( linearMove ) * sampleGaussian( srr ) + fabs( angularMove ) * sampleGaussian( str );
	double am=angularMove + fabs( linearMove ) * sampleGaussian( srt ) + fabs( angularMove ) * sampleGaussian( stt );
	n.x+=lm*cos(n.theta+.5*am);
	n.y+=lm*sin(n.theta+.5*am);
	n.theta+=am;
	n.theta=atan2(sin(n.theta), cos(n.theta));
	return n;
}

/*
@desc 里程计运动模型
@p    表示粒子估计的最优位置(机器人上一个时刻的最优位置)
@pnew 表示里程计算出来的新的位置
@pold 表示里程计算出来的旧的位置(即上一个里程计的位置)
*/
OrientedPoint 
MotionModel::drawFromMotion(const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold) const{
	double sxy=0.3*srr;
    /*
     * 计算出pnew 相对于 pold走了多少距离
     * 这里的距离表达是相对于车身坐标系来说的。
    */
	OrientedPoint delta=absoluteDifference(pnew, pold);
	
	/*初始化一个点*/
	OrientedPoint noisypoint(delta);
	
	/*走过的X轴方向的距离加入噪声*/
	noisypoint.x+=sampleGaussian(srr*fabs(delta.x)+str*fabs(delta.theta)+sxy*fabs(delta.y));
	
	/*走过的Y轴方向的距离加入噪声*/
	noisypoint.y+=sampleGaussian(srr*fabs(delta.y)+str*fabs(delta.theta)+sxy*fabs(delta.x));
	
	/*走过的Z轴方向的距离加入噪声*/
	noisypoint.theta+=sampleGaussian(stt*fabs(delta.theta)+srt*sqrt(delta.x*delta.x+delta.y*delta.y));
	
	/*限制角度的范围*/
	noisypoint.theta=fmod(noisypoint.theta, 2*M_PI);
	if (noisypoint.theta>M_PI)
		noisypoint.theta-=2*M_PI;
	
	/*把加入了噪声的值 加到粒子估计的最优的位置上  即得到新的位置(根据运动模型推算出来的位置)*/
	return absoluteSum(p,noisypoint);
}


/*
OrientedPoint 
MotionModel::drawFromMotion(const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold) const{
	
	//compute the three stps needed for perfectly matching the two poses if the noise is absent
	
	OrientedPoint delta=pnew-pold;
	double aoffset=atan2(delta.y, delta.x);
	double alpha1=aoffset-pold.theta;
	alpha1=atan2(sin(alpha1), cos(alpha1));
	double rho=sqrt(delta*delta);
	double alpha2=pnew.theta-aoffset;
	alpha2=atan2(sin(alpha2), cos(alpha2));
	
	OrientedPoint pret=drawFromMotion(p, 0, alpha1);
	pret=drawFromMotion(pret, rho, 0);
	pret=drawFromMotion(pret, 0, alpha2);
	return pret;
}
*/


Covariance3 MotionModel::gaussianApproximation(const OrientedPoint& pnew, const OrientedPoint& pold) const
{
	OrientedPoint delta=absoluteDifference(pnew,pold);
	
	/*两个位置的线性位移和角度位移*/
	double linearMove=sqrt(delta.x*delta.x+delta.y*delta.y);
	double angularMove=fabs(delta.x);
	
	
	double s11=srr*srr*linearMove*linearMove;
	double s22=stt*stt*angularMove*angularMove;
	double s12=str*angularMove*srt*linearMove;
	
	Covariance3 cov;
	double s=sin(pold.theta),c=cos(pold.theta);
	
	cov.xx=c*c*s11+MotionModelConditioningLinearCovariance;
	cov.yy=s*s*s11+MotionModelConditioningLinearCovariance;
	cov.tt=s22+MotionModelConditioningAngularCovariance;
	cov.xy=s*c*s11;
	cov.xt=c*s12;
	cov.yt=s*s12;
	
	return cov;
}

};

