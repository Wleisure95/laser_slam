#ifndef ODOMETRYSENSOR_H
#define ODOMETRYSENSOR_H

#include <string>
#include "../sensor_base/sensor.h"

namespace GMapping{

//里程计传感器  
class OdometrySensor: public Sensor{
	public:
		OdometrySensor(const std::string& name, bool ideal=false);
		inline bool isIdeal() const { return m_ideal; }
	protected:
		bool m_ideal;			//标明是否是理想的传感器
};

};

#endif

