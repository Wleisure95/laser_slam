#ifndef SENSOR_H
#define SENSOR_H

#include <string>
#include <map>

namespace GMapping{

//传感器的基类  传入传感器的名字
//这个基类的作用就是储存传感器的名字
class Sensor
{
	public:
		Sensor(const std::string& name="");
		virtual ~Sensor();
		inline std::string getName() const {return m_name;}
		inline void setName(const std::string& name) {m_name=name;}
	protected:
		std::string m_name;
};

//传感器的地图，即车子上有多少传感器都可以存入地图里面
typedef std::map<std::string, Sensor*> SensorMap;

}; //end namespace

#endif

