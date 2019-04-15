#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include <vector>
#include "../sensor_base/sensor.h"
#include "../../utils/point.h"

namespace GMapping{

/*
 * 激光传感器类
 * 这里面存储了激光传感器相对于base_link的位姿。
 * 以及激光传感器的每个激光束的角度 和 这个角度对应的sin cos值
 * 以及激光传感器的激光束的数量。
 *
 *
 * 因为gmapping算法的初始使用的激光雷达传感器都是等角度的传感器。
 * 因此这个类主要用来描述这个激光雷达传感器对应的参数特性
 *
 * 也就是说我们做修改的话，需要改变这里激光束角度是均匀分布的
 *
 *
*/
class RangeSensor: public Sensor
{
	friend class Configuration;
	friend class CarmenConfiguration;
	friend class CarmenWrapper;
	public:

        /*这个结构体表示一束激光束*/
        struct Beam
        {
            OrientedPoint pose;	//pose relative to the center of the sensor 相对于传感器中心的坐标(一般来说，坐标为0 但是角度不为0)
            double span;	//spam=0 indicates a line-like beam             0 线激光
            double maxRange;	//maximum range of the sensor               传感器的最大测量范围
            double s,c;		//sinus and cosinus of the beam (optimization); 这个激光的sin和cos值
		};	
		/*构造函数 给激光命令*/
		RangeSensor(std::string name);
		
		/**/
		RangeSensor(std::string name, unsigned int beams, double res, const OrientedPoint& position=OrientedPoint(0,0,0), double span=0, double maxrange=89.0);
        RangeSensor(std::string name,unsigned int beams_num,double *angles,const OrientedPoint& position=OrientedPoint(0,0,0),double span=0,double maxrange=89);
		
		/*返回所有的激光数据(常量)*/
		inline const std::vector<Beam>& beams() const {return m_beams;}
		
		/*返回所有的激光数据*/
		inline std::vector<Beam>& beams() {return m_beams;}
		
		/*返回激光传感器的位姿*/
		inline OrientedPoint getPose() const {return m_pose;}
		
		/*更新激光查询表*/
		void updateBeamsLookup();
		bool newFormat;
	protected:
		OrientedPoint m_pose;		//激光传感器的位姿
		std::vector<Beam> m_beams;  //激光数据
};

};

#endif
