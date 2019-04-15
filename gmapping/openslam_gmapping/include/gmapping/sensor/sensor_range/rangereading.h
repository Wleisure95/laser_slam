#ifndef RANGEREADING_H
#define RANGEREADING_H

#include <vector>
#include "../include/gmapping/sensor/sensor_base/sensorreading.h"
#include "../include/gmapping/sensor/sensor_range/rangesensor.h"

namespace GMapping{

/*
 * 激光传感器数据类
 * 这个类集自传感器数据类
 * 也就说从传感器数据类(SebsirReading)里面继承了两个变量 传感器的名字 + 数据的时间戳
 *
 * 这个类相当于用集成自vector的数组来储存激光的距离数据。
 * 然后用RangeSensor这个类来储存激光的角度数据
 * 实际上角度数据 也可以储存在这里面
 *
 */
class RangeReading: public SensorReading
{
	public:
		
		/*构造函数*/
		RangeReading(const RangeSensor* rs, double time=0);
		
		/*构造函数*/
		RangeReading(unsigned int n_beams, const double* d, const RangeSensor* rs, double time=0);

        RangeReading(unsigned int n_beams, const double*d,const double*angles,const RangeSensor*rs,double time=0);
		
		virtual ~RangeReading();
		
        //得到这帧激光数据的位置
		inline const OrientedPoint& getPose() const {return m_pose;}

        //返回激光束的多少
        inline const unsigned int getSize() const {return m_beams;}
		
        //设置这帧传感器数据的位置
		inline void setPose(const OrientedPoint& pose) {m_pose=pose;}
		
		unsigned int rawView(double* v, double density=0.) const;
		
		std::vector<Point> cartesianForm(double maxRange=1e6) const;
		
		unsigned int activeBeams(double density=0.) const;


        //经常需要外部访问 所以设置为public

        //存储激光雷达的距离信息
        std::vector<double>m_dists;

        unsigned int m_beams;

        //每个激光束对应的角度
        std::vector<double> m_angles;


	protected:
        //这帧激光数据的位置 这里的位置表示的是机器人的位姿 不是激光雷达的位姿
		OrientedPoint m_pose;
};

//自己定义的激光雷达数据的
class ChampionLaserScan: public SensorReading
{
public:
    ChampionLaserScan(const RangeSensor*rs,double time=0);

    ChampionLaserScan(const double* dists, const double *angles,unsigned int n_beams, double time=0);

    virtual ~ChampionLaserScan();

    inline const OrientedPoint& getPose() const {return m_pose;}

    inline void setPose(const OrientedPoint& pose){m_pose = pose;}

    unsigned int rawView(double *v,double density = 0.) const;

    std::vector<Point> cartesianForm(double maxRange=1e6) const;

    unsigned int activeBeams(double density = 0.) const;

protected:
    OrientedPoint m_pose;
    std::vector<double> beam_dists;
    std::vector<double> beam_angles;
};

};

#endif
