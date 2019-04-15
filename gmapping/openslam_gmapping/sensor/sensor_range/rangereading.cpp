#include <limits>
#include <iostream>
#include <assert.h>
#include <sys/types.h>
#include "../include/gmapping/utils/gvalues.h"
#include "../include/gmapping/sensor/sensor_range/rangereading.h"

namespace GMapping{

using namespace std;


/*构造函数*/
RangeReading::RangeReading(const RangeSensor* rs, double time):
	SensorReading(rs,time){}

/*构造函数
n_beams  指定了激光束的数量
d        表示各个激光的距离
*/
RangeReading::RangeReading(unsigned int n_beams, const double* d, const RangeSensor* rs, double time):
	SensorReading(rs,time)
{
	assert(n_beams==rs->beams().size());
    m_beams = n_beams;
    m_dists.resize(n_beams);
    for (unsigned int i=0; i<n_beams; i++)
        m_dists[i]=d[i];
}

//这个和上面的不同在与 这个会存储角度信息。
RangeReading::RangeReading(unsigned int n_beams, const double*d,const double*angles,const RangeSensor*rs,double time):
    SensorReading(rs,time)
{
    m_beams = n_beams;
    m_dists.resize(n_beams);
    m_angles.resize(n_beams);
    //cerr <<"RangeReading"<<endl;
    for(unsigned int i = 0; i<n_beams;i++)
    {
        m_dists[i]=d[i];
        m_angles[i] = angles[i];
    }
}



RangeReading::~RangeReading(){
//	cerr << __PRETTY_FUNCTION__ << ": CAZZZZZZZZZZZZZZZZZZZZOOOOOOOOOOO" << endl;
}

/*
返回经过density过滤之后的激光束的值
其中被过滤掉的激光的值，都被赋值为MAX
*/
unsigned int RangeReading::rawView(double* v, double density) const
{
	//如果density等于0 则所有激光束全部合法
	if (density==0)
	{
        for (unsigned int i=0; i<m_dists.size(); i++)
            v[i]=m_dists[i];
	}
	else 
	{
		Point lastPoint(0,0);
		uint suppressed=0;
        for (unsigned int i=0; i<m_dists.size(); i++)
		{
			const RangeSensor* rs=dynamic_cast<const RangeSensor*>(getSensor());
			assert(rs);
			Point lp(cos(m_angles[i])*m_dists[i],sin(m_angles[i])*m_dists[i]);

			Point dp=lastPoint-lp;
			double distance=sqrt(dp*dp);
			/*只有两束激光的距离大于某一个值的之后，才是合法的*/
			if (distance<density)
			{
				v[i]=std::numeric_limits<double>::max();
				suppressed++;
			}
			else
			{
				lastPoint=lp;
                v[i]=m_dists[i];
			}
		}
	}
	//	return size();
    return static_cast<unsigned int>(m_dists.size());

};

/*
计算在指定激光密度的时候，有多少的有效激光束
density表示两束激光的端点距离必须大于density
返回有效的激光的数量(如果两束激光过于接近，则视为无效)
这个函数从来没有被使用过！！！！！！
*/
unsigned int RangeReading::activeBeams(double density) const
{
	if (density==0.)
        return m_dists.size();

    int ab=0;
	Point lastPoint(0,0);
	uint suppressed=0;
    for (unsigned int i=0; i < m_dists.size(); i++)
	{
		const RangeSensor* rs=dynamic_cast<const RangeSensor*>(getSensor());
		assert(rs);
		Point lp(
            cos(rs->beams()[i].pose.theta)*m_dists[i],
            sin(rs->beams()[i].pose.theta)*m_dists[i]);
		Point dp=lastPoint-lp;
		double distance=sqrt(dp*dp);
		/*两束激光过于接近 则视为无效*/
		if (distance<density)
		{
			suppressed++;
		}
		/*否则认为有效激光数量+1*/
		else
		{
			lastPoint=lp;
			ab++;
		}
		//std::cerr<< __PRETTY_FUNCTION__ << std::endl;
		//std::cerr<< "suppressed " << suppressed <<"/"<<size() << std::endl;
	}
	return ab;
}

/*
 * 把激光数据转换为在笛卡尔坐标系下的数据
 * 也就是说把所有的激光数据转换成点云
 * 这个函数从来没有被使用过！！！！！！！
*/
std::vector<Point> RangeReading::cartesianForm(double maxRange) const
{
	const RangeSensor* rangeSensor=dynamic_cast<const RangeSensor*>(getSensor());
	assert(rangeSensor && rangeSensor->beams().size());
	//	uint m_beams=rangeSensor->beams().size();
	uint m_beams=static_cast<unsigned int>(rangeSensor->beams().size());
	std::vector<Point> cartesianPoints(m_beams);
	double px,py,ps,pc;
	px=rangeSensor->getPose().x;
	py=rangeSensor->getPose().y;
	ps=sin(rangeSensor->getPose().theta);
	pc=cos(rangeSensor->getPose().theta);
	for (unsigned int i=0; i<m_beams; i++)
	{
        const double& rho=m_dists[i];
		const double& s=rangeSensor->beams()[i].s;
		const double& c=rangeSensor->beams()[i].c;
		if (rho>=maxRange)
		{
			cartesianPoints[i]=Point(0,0);
		} 
		else 
		{
			Point p=Point(rangeSensor->beams()[i].pose.x+c*rho, rangeSensor->beams()[i].pose.y+s*rho);
			cartesianPoints[i].x=px+pc*p.x-ps*p.y;
			cartesianPoints[i].y=py+ps*p.x+pc*p.y;
		}
	}
	return cartesianPoints;
}

};

