#ifndef SENSORREADING_H
#define SENSORREADING_H

#include "sensor.h"
namespace GMapping{

/*
 * 传感器数据类
 * 这个类也是一个基类 作用是存储传感器的名字+传感器读取的时间
*/
class SensorReading{
	public:
		SensorReading(const Sensor* s=0, double time=0);
		virtual ~SensorReading();
		inline double getTime() const {return m_time;}
		inline void setTime(double t) {m_time=t;}
		inline const Sensor* getSensor() const {return m_sensor;}
	protected:
		double m_time;
		const Sensor* m_sensor;

};

}; //end namespace
#endif


