#ifndef SMMAP_H
#define SMMAP_H
#include "../include/gmapping/grid/map.h"
#include "../include/gmapping/grid/harray2d.h"
#include "../include/gmapping/utils/point.h"
#define SIGHT_INC 1

namespace GMapping {

/*表示地图中的一个cell*/
struct PointAccumulator
{
	typedef point<float> FloatPoint;
	/* before 
	PointAccumulator(int i=-1): acc(0,0), n(0), visits(0){assert(i==-1);}
	*/
	/*after begin*/
	PointAccumulator(): acc(0,0), n(0), visits(0){}
	PointAccumulator(int i): acc(0,0), n(0), visits(0){assert(i==-1);}
	/*after end*/
    inline void update(bool value, const Point& p=Point(0,0));
	
	inline Point mean() const {return 1./n*Point(acc.x, acc.y);}
	
	/*返回被占用的概率*/
	inline operator double() const { return visits?(double)n*SIGHT_INC/(double)visits:-1; }
	
	inline void add(const PointAccumulator& p) {acc=acc+p.acc; n+=p.n; visits+=p.visits; }
	
	static const PointAccumulator& Unknown();
	
	static PointAccumulator* unknown_ptr;
	
    /*
     * cell的位置的累计值 这里认为每个cell算被击中的时候，有可能不是在同一个位置被击中的
     * 毕竟在查找对应点的时候是在一个kernelSize的窗口里面进行查找的，所以这里累计被击中的坐标值。
     * 而这个cell的真实坐标由这些累计值的均值表示
     */
	FloatPoint acc;
	
	/*n表示被hit中的次数  visits表示访问的次数*/
	int n, visits;
	inline double entropy() const;
};

/*
@desc 更新某个Cell的状态

如果被击中 则需要对cell的值进行累加

@value 表示是否被击中
@p     表示被击中的坐标  世界坐标系
*/
void PointAccumulator::update(bool value, const Point& p)
{
	if (value) 
	{
		acc.x+= static_cast<float>(p.x);
		acc.y+= static_cast<float>(p.y); 
		n++; 
		visits+=SIGHT_INC;
    }
    else
		visits++;
}

/*
#@desc 该点的熵，表示该点被击中的概率对应的熵。
*/
double PointAccumulator::entropy() const
{
	if (!visits)
		return -log(.5);
	if (n==visits || n==0)
		return 0;

	double x=(double)n*SIGHT_INC/(double)visits;
	return -( x*log(x)+ (1-x)*log(1-x) );
}

//定义最终使用的地图数据类型 cell类型为PointAccumulator 存储数据类型为HierarchicalArray2D<PointAccumulator>
typedef Map<PointAccumulator,HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;

};

#endif 
