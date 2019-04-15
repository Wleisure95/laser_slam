#include <gmapping/sensor/sensor_range/rangesensor.h>

namespace GMapping{

RangeSensor::RangeSensor(std::string name): Sensor(name){}

/*
beams_num   激光束的数量
res			激光的角度解析度
position    激光的位置
span        表示激光束的发散角度(激光雷达为0)
maxRange    激光束的最大距离
*/
RangeSensor::RangeSensor(std::string name, unsigned int beams_num, double res, const OrientedPoint& position, double span, double maxrange):Sensor(name),
    m_pose(position), m_beams(beams_num)
{
	double angle=-.5*res*beams_num;
    for (unsigned int i=0; i<beams_num; i++, angle+=res)
    {
		RangeSensor::Beam& beam(m_beams[i]);
		beam.span=span;
		beam.pose.x=0;
		beam.pose.y=0;
		beam.pose.theta=angle;
		beam.maxRange=maxrange;
	}
	newFormat=0;
	updateBeamsLookup();
}


RangeSensor::RangeSensor(std::string name,unsigned int beams_num,double *angles,const OrientedPoint& position,double span,double maxrange):Sensor(name),
    m_pose(position),m_beams(beams_num)
{
    for(int i = 0; i<beams_num;i++)
    {
        RangeSensor::Beam& beam(m_beams[i]);
        beam.span = span;
        beam.pose.x = 0;
        beam.pose.y = 0;
        beam.pose.theta = angles[i];
        beam.maxRange = maxrange;
    }
    newFormat = 0;
    updateBeamsLookup();
}

/*计算查询表，即计算每个激光束对应的角度的正弦、余弦值*/
void RangeSensor::updateBeamsLookup(){
    for (unsigned int i=0; i<m_beams.size(); i++)
    {
		RangeSensor::Beam& beam(m_beams[i]);
		beam.s=sin(m_beams[i].pose.theta);
		beam.c=cos(m_beams[i].pose.theta);
	}
}

};
