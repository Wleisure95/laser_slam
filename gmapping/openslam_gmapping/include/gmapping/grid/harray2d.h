#ifndef HARRAY2D_H
#define HARRAY2D_H
#include <set>
#include <gmapping/utils/point.h>
#include <gmapping/utils/autoptr.h>
#include "array2d.h"

namespace GMapping
{

/*
 * PS:这里的cell在gmapping里面指的是PointAccumulator
 * 这里是地图的类。这个类最终在地图类Map(map.h)中被使用
 * 这个是继承自Array2D<Array2D<Cell>>
 * 也就是说这个类里面也有一个m_cells的变量，变量的类型为指向Array2D<Cell>的二维指针
 * 也就是说HierarchicalArray2D是一个二维的patch数组
 *
 * 地图里面有patch的概念，按照GMAPPING的做法，一个patch就是一个地图块，实际上就是一个Array2D<Cell>的指针。
 * 这一个patch里面包含有2^m_patchMagnitude * 2^m_patchMagnitude 个cell。
 * 也就是说地图实际上是分为两层的，第一层的元素为Patch 第二层的元素为cell
 * 首先把HierarchicalArray2D的元素为patch(Array2D<cell>)，然后patch的元素为cell。
 *
patch的大小等级由 m_patchMagnitude决定

patch的实际大小等于 m_patchSize=1<<m_patchMagnitude
patch的实际大小表示一个patch里面包含有m_patchSize*m_patchSize的cell

在gmapping源码中 m_patchMagnitude的默认大小为5 也就是说patch的默认大小为32*32

*/
template <class Cell>
class HierarchicalArray2D: public Array2D<autoptr< Array2D<Cell> > >
{
	public:
		typedef std::set< point<int>, pointcomparator<int> > PointSet;

        //对应的构造函数
		HierarchicalArray2D(int xsize, int ysize, int patchMagnitude=5);
		HierarchicalArray2D(const HierarchicalArray2D& hg);
		HierarchicalArray2D& operator=(const HierarchicalArray2D& hg);
		
		virtual ~HierarchicalArray2D(){}
		
		void resize(int ixmin, int iymin, int ixmax, int iymax);
		inline int getPatchSize() const {return m_patchMagnitude;}
		inline int getPatchMagnitude() const {return m_patchMagnitude;}
		
		/*这些cell函数，调用的是父类Array2D类中的函数*/
		inline const Cell& cell(int x, int y) const;
		inline Cell& cell(int x, int y);
		inline bool isAllocated(int x, int y) const;
		inline AccessibilityState cellState(int x, int y) const ;
		inline IntPoint patchIndexes(int x, int y) const;
		
		inline const Cell& cell(const IntPoint& p) const { return cell(p.x,p.y); }
		inline Cell& cell(const IntPoint& p) { return cell(p.x,p.y); }
		
		inline bool isAllocated(const IntPoint& p) const { return isAllocated(p.x,p.y);}
		inline AccessibilityState cellState(const IntPoint& p) const { return cellState(p.x,p.y); }
		inline IntPoint patchIndexes(const IntPoint& p) const { return patchIndexes(p.x,p.y);}
		
		inline void setActiveArea(const PointSet&, bool patchCoords=false);
		const PointSet& getActiveArea() const {return m_activeArea; }
		inline void allocActiveArea();
	protected:
		virtual Array2D<Cell> * createPatch(const IntPoint& p) const;
		PointSet m_activeArea;  //存储地图中使用到的Cell的坐标
		
		int m_patchMagnitude;   //patch的大小等级  
        int m_patchSize;		//patch的实际大小 表示pathch里面包含的cell的个数 m_patchSize=1<<m_patchMagnitude;
};

/*
@desc 构造函数，创建一个HierarchicalArray2D,大小为xsize*ysize 等级为patchMagnitude
则这个HierarchicalArray2D的内部的patch的大小为xsize>>patchMagnitude * ysize>>patchMagnitude
*/
template <class Cell>
HierarchicalArray2D<Cell>::HierarchicalArray2D(int xsize, int ysize, int patchMagnitude) 
  :Array2D<autoptr< Array2D<Cell> > >::Array2D((xsize>>patchMagnitude), (ysize>>patchMagnitude))
{
	m_patchMagnitude=patchMagnitude;
	m_patchSize=1<<m_patchMagnitude;
}

template <class Cell>
HierarchicalArray2D<Cell>::HierarchicalArray2D(const HierarchicalArray2D& hg)
  :Array2D<autoptr< Array2D<Cell> > >::Array2D((hg.m_xsize>>hg.m_patchMagnitude), (hg.m_ysize>>hg.m_patchMagnitude))  // added by cyrill: if you have a resize error, check this again
{
	this->m_xsize=hg.m_xsize;
	this->m_ysize=hg.m_ysize;
	this->m_cells=new autoptr< Array2D<Cell> >*[this->m_xsize];
	for (int x=0; x<this->m_xsize; x++){
		this->m_cells[x]=new autoptr< Array2D<Cell> >[this->m_ysize];
		for (int y=0; y<this->m_ysize; y++)
			this->m_cells[x][y]=hg.m_cells[x][y];
	}
	this->m_patchMagnitude=hg.m_patchMagnitude;
	this->m_patchSize=hg.m_patchSize;
}

/**
 *重新扩充地图的大小
 */
template <class Cell>
void HierarchicalArray2D<Cell>::resize(int xmin, int ymin, int xmax, int ymax)
{
	int xsize=xmax-xmin;
	int ysize=ymax-ymin;
    //分配一个xsize*ysize大小的patch的内存
	autoptr< Array2D<Cell> > ** newcells=new autoptr< Array2D<Cell> > *[xsize];
	for (int x=0; x<xsize; x++)
	{
		newcells[x]=new autoptr< Array2D<Cell> >[ysize];
		for (int y=0; y<ysize; y++)
		{
			newcells[x][y]=autoptr< Array2D<Cell> >(0);
		}
	}
	
    //把原来地图中存在的数据 拷贝到新分配的地图中 这样就完成了地图扩充
	int dx= xmin < 0 ? 0 : xmin;
	int dy= ymin < 0 ? 0 : ymin;
	int Dx=xmax<this->m_xsize?xmax:this->m_xsize;
	int Dy=ymax<this->m_ysize?ymax:this->m_ysize;
	
	for (int x=dx; x<Dx; x++)
	{
		for (int y=dy; y<Dy; y++)
		{
			newcells[x-xmin][y-ymin]=this->m_cells[x][y];
		}
		delete [] this->m_cells[x];
	}
	delete [] this->m_cells;
	this->m_cells=newcells;
	this->m_xsize=xsize;
	this->m_ysize=ysize; 
}

/**
 *两个地图的赋值运算符的重载
 */
template <class Cell>
HierarchicalArray2D<Cell>& HierarchicalArray2D<Cell>::operator=(const HierarchicalArray2D& hg)
{
//	Array2D<autoptr< Array2D<Cell> > >::operator=(hg);
    //如果复制的两个地图的大小不一样 则需要把目前的地图删除，然后重新复制为新的地图

    //删除当前地图 并且分配内存
    if (this->m_xsize!=hg.m_xsize || this->m_ysize!=hg.m_ysize)
    {
		for (int i=0; i<this->m_xsize; i++)
			delete [] this->m_cells[i];
		delete [] this->m_cells;

		this->m_xsize=hg.m_xsize;
		this->m_ysize=hg.m_ysize;
		this->m_cells=new autoptr< Array2D<Cell> >*[this->m_xsize];
		for (int i=0; i<this->m_xsize; i++)
			this->m_cells[i]=new autoptr< Array2D<Cell> > [this->m_ysize];
	}

    //赋值为新的地图
	for (int x=0; x<this->m_xsize; x++)
    {
        for (int y=0; y<this->m_ysize; y++)
        {
            this->m_cells[x][y]=hg.m_cells[x][y];
        }
    }
	
	m_activeArea.clear();
	m_patchMagnitude=hg.m_patchMagnitude;
	m_patchSize=hg.m_patchSize;
	return *this;
}


/*
@desc 设置地图的有效区域
pathCoords指示当前的PointSet的坐标是否是对应的Patch坐标，如果不是则需要转换到pathch坐标然后再插入
对于HierarchicalArray2D来说，考虑的尺度都是patch。因此进行扩充地图或者分配内存的时候，每次也都是以patch为单位的。
因此如果插入的点的坐标不是patch的坐标，则需要转换为对应的patch坐标，然后再插入。

注意：这里只是把点存储到对应的队列中，并没有进行内存的分配，真正的内存分配在后面的allocActiveArea()函数

*/
template <class Cell>
void HierarchicalArray2D<Cell>::setActiveArea(const typename HierarchicalArray2D<Cell>::PointSet& aa, bool patchCoords)
{
	m_activeArea.clear();
	for (PointSet::const_iterator it= aa.begin(); it!=aa.end(); ++it) 
	{
		IntPoint p;
		if (patchCoords)
			p=*it;
		else
			p=patchIndexes(*it);
		m_activeArea.insert(p);
	}
}

/*
@一个patch表示一个Array2D
patch的大小有m_patchMagnitude指定
*/
template <class Cell>
Array2D<Cell>* HierarchicalArray2D<Cell>::createPatch(const IntPoint& ) const
{
	return new Array2D<Cell>(1<<m_patchMagnitude, 1<<m_patchMagnitude);
}


/*
@desc 返回对应的cell状态
要访问某个具体的cell。

首先转换到patch坐标系
再从对应的patch中访问对应的cell元素
*/
template <class Cell>
AccessibilityState  HierarchicalArray2D<Cell>::cellState(int x, int y) const 
{
	if (this->isInside(patchIndexes(x,y))) 
	{
		if(isAllocated(x,y))
			return (AccessibilityState)((int)Inside|(int)Allocated);
		else
			return Inside;
	}
	return Outside;
}

/*
@desc 给有效区域(被激光扫过的区域)分配内存
给setActiveArea()函数插入的patch进行内存的分配。
如果插入的区域没有分配内存，则进行分配。
如果已经分配了内存则不再分配
*/
template <class Cell>
void HierarchicalArray2D<Cell>::allocActiveArea()
{
	for (PointSet::const_iterator it= m_activeArea.begin(); it!=m_activeArea.end(); ++it)
	{
		const autoptr< Array2D<Cell> >& ptr=this->m_cells[it->x][it->y];
		Array2D<Cell>* patch=0;
        //如果对应的active没有被分配内存 则进行内存分配
		if (!ptr)
		{
			patch=createPatch(*it);
		} 
        //如果已经分配了还是赋原值
		else
		{	
			patch=new Array2D<Cell>(*ptr);
		}
		this->m_cells[it->x][it->y]=autoptr< Array2D<Cell> >(patch);
	}
}

/*
@desc 判断坐标为xy的cell是否被分配了内存
*/
template <class Cell>
bool HierarchicalArray2D<Cell>::isAllocated(int x, int y) const
{
	IntPoint c=patchIndexes(x,y);
	autoptr< Array2D<Cell> >& ptr=this->m_cells[c.x][c.y];
	return (ptr != 0);
}

/*
@desc 把原始坐标XY，转换为对应的patch的坐标
原始坐标转换为patch坐标相当于把原始坐标/32
*/
template <class Cell>
IntPoint HierarchicalArray2D<Cell>::patchIndexes(int x, int y) const
{
	if (x>=0 && y>=0)
		return IntPoint(x>>m_patchMagnitude, y>>m_patchMagnitude);
	return IntPoint(-1, -1);
}


/*
@desc 返回对应位置的x,y位置的Cell--pointAccu
*/
template <class Cell>
Cell& HierarchicalArray2D<Cell>::cell(int x, int y)
{
	IntPoint c=patchIndexes(x,y);
	assert(this->isInside(c.x, c.y));
	if (!this->m_cells[c.x][c.y])
	{
		Array2D<Cell>* patch=createPatch(IntPoint(x,y));
		this->m_cells[c.x][c.y]=autoptr< Array2D<Cell> >(patch);
		//cerr << "!!! FATAL: your dick is going to fall down" << endl;
	}
	autoptr< Array2D<Cell> >& ptr=this->m_cells[c.x][c.y];
	return (*ptr).cell(IntPoint(x-(c.x<<m_patchMagnitude),y-(c.y<<m_patchMagnitude)));
}


template <class Cell>
const Cell& HierarchicalArray2D<Cell>::cell(int x, int y) const
{
	assert(isAllocated(x,y));
	IntPoint c=patchIndexes(x,y);
	const autoptr< Array2D<Cell> >& ptr=this->m_cells[c.x][c.y];
	return (*ptr).cell(IntPoint(x-(c.x<<m_patchMagnitude),y-(c.y<<m_patchMagnitude)));
}

};

#endif

