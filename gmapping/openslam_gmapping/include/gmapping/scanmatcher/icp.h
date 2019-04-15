#ifndef _ICP_H_
#define _ICP_H_

#include "../include/gmapping/utils/point.h"
#include <utility>
#include <list>
#include <vector>


/*
 * 这个文件里面的两个函数的功能都是一样的。
 * 都是假设知道了两坨点云的匹配关系，来求解得到这两坨点云之间的转换关系
 * PointPairContainer 存储两坨一一对应的点云数据
*/
namespace GMapping{
typedef std::pair<Point,Point> PointPair;

//用来进行ICP优化的中的某一步 这里计算出来的是first 到 second的转换。
//也就是说second = T*first;
template <typename PointPairContainer>
double icpLinearStep(OrientedPoint & retval, const PointPairContainer& container)
{
	typedef typename PointPairContainer::const_iterator ContainerIterator;
	PointPair mean=std::make_pair(Point(0.,0.), Point(0.,0.));
	int size=0;

    //求两坨点云的几何中心
    for (ContainerIterator it=container.begin(); it!=container.end(); it++)
    {
		mean.first=mean.first+it->first;
		mean.second=mean.second+it->second;
		size++;
	}
	mean.first=mean.first*(1./size);
	mean.second=mean.second*(1./size);
	double sxx=0, sxy=0, syx=0, syy=0;
	
    //对两坨点云都减去各自的几何中心
    for (ContainerIterator it=container.begin(); it!=container.end(); it++)
    {
        //减去各自的集合中心
		PointPair mf=std::make_pair(it->first-mean.first, it->second-mean.second);

        //累加xi*yi^T
        sxx+=mf.first.x*mf.second.x;
		sxy+=mf.first.x*mf.second.y;
		syx+=mf.first.y*mf.second.x;
		syy+=mf.first.y*mf.second.y;
	}

    //计算两坨点云之间的转换关系 这里是根据SVD分解得到的矩阵
    //这里得到的转换关系表示 第一坨点云 到 第二坨点云 的转换关系
	retval.theta=atan2(sxy-syx, sxx+sxy);
	double s=sin(retval.theta), c=cos(retval.theta);
	retval.x=mean.second.x-(c*mean.first.x-s*mean.first.y);
	retval.y=mean.second.y-(s*mean.first.x+c*mean.first.y);
	
    //统计这个转换关系的对应的误差
	double error=0;
    for (ContainerIterator it=container.begin(); it!=container.end(); it++)
    {
		Point delta(
			c*it->first.x-s*it->first.y+retval.x-it->second.x, s*it->first.x+c*it->first.y+retval.y-it->second.y);
		error+=delta*delta;
	}
	return error;	
}

/*
 * 这个函数的功能没看懂
 * 用来进行非线性ICP优化中的某一步
 * //也就是说second = T*first;
*/

template <typename PointPairContainer>
double icpNonlinearStep(OrientedPoint & retval, const PointPairContainer& container)
{
	typedef typename PointPairContainer::const_iterator ContainerIterator;
	PointPair mean=std::make_pair(Point(0.,0.), Point(0.,0.));
	int size=0;
    //求解得到两坨点云的几何中心
	for (ContainerIterator it=container.begin(); it!=container.end(); it++)
	{
		mean.first=mean.first+it->first;
		mean.second=mean.second+it->second;
		size++;
	}
	mean.first=mean.first*(1./size);
	mean.second=mean.second*(1./size);
	

    //两坨点云中的数据各自减去自己的几何中心
	double ms=0,mc=0;
	for (ContainerIterator it=container.begin(); it!=container.end(); it++)
	{
		PointPair mf=std::make_pair(it->first-mean.first, it->second-mean.second);
		double  dalpha=atan2(mf.second.y, mf.second.x) - atan2(mf.first.y, mf.first.x);
		double gain=sqrt(mean.first*mean.first);
		ms+=gain*sin(dalpha);
		mc+=gain*cos(dalpha);
	}

    //计算这两坨点云的转换关系 第一坨点云p1到第二坨点云p2的转换关系。
    //也就是说 P2 = TC*P1;
	retval.theta=atan2(ms, mc);
	double s=sin(retval.theta), c=cos(retval.theta);

    //平移通过旋转之后的两坨点云的质心之间的位移来计算
	retval.x=mean.second.x-(c*mean.first.x-s*mean.first.y);
	retval.y=mean.second.y-(s*mean.first.x+c*mean.first.y);
	
    //这算这个转换关系对应的误差
	double error=0;
	for (ContainerIterator it=container.begin(); it!=container.end(); it++)
	{
		Point delta(
			c*it->first.x-s*it->first.y+retval.x-it->second.x, s*it->first.x+c*it->first.y+retval.y-it->second.y);
		error+=delta*delta;
	}
	return error;	
}

}//end namespace

#endif
