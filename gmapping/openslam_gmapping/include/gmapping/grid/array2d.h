#ifndef ARRAY2D_H
#define ARRAY2D_H

#include <assert.h>
#include <gmapping/utils/point.h>
#include "accessstate.h"

#include <iostream>

#ifndef __PRETTY_FUNCTION__
#define __FUNCDNAME__
#endif

namespace GMapping {

/*
 * Array2D类 可以理解为二维的数组，数组的模板元素为Cell类。
 * 在gmapping的使用过程中cell类的实例是PointAccumulator
二维数组
模板是Cell类

对于DoubleArray2D来说 模板是double
*/
template<class Cell, const bool debug=false> class Array2D
{
	public:
		/*构造函数*/
		Array2D(int xsize=0, int ysize=0);
		Array2D& operator=(const Array2D &);
		Array2D(const Array2D<Cell,debug> &);
		~Array2D();
		
		/*清除和重定义大小*/
		void clear();
		void resize(int xmin, int ymin, int xmax, int ymax);
		
		/*判断坐标是否在数组类*/
		inline bool isInside(int x, int y) const;
		
		/*返回XY处的Cell*/
		inline const Cell& cell(int x, int y) const;
		
		/*返回XY处的Cell*/
		inline Cell& cell(int x, int y);
		
		/*判断某个cell的状态*/
		inline AccessibilityState cellState(int x, int y) const { return (AccessibilityState) (isInside(x,y)?(Inside|Allocated):Outside);}
		
		/*功能和上面的四个相同*/
		inline bool isInside(const IntPoint& p) const { return isInside(p.x, p.y);}
		inline const Cell& cell(const IntPoint& p) const {return cell(p.x,p.y);}
		inline Cell& cell(const IntPoint& p) {return cell(p.x,p.y);}
		inline AccessibilityState cellState(const IntPoint& p) const { return cellState(p.x, p.y);}
		
        //因为这里每一个元素都是不可再分的，因此PatchSize都是0
		inline int getPatchSize() const{return 0;}
		inline int getPatchMagnitude() const{return 0;}
		
		/*返回地图大小和元素*/
		inline int getXSize() const {return m_xsize;}
		inline int getYSize() const {return m_ysize;}
		inline Cell** cells() {return m_cells;}
		
		/*存储的cell元素*/
		Cell ** m_cells;
	protected:
		int m_xsize, m_ysize;
};


/*
@desc 构造函数，给Cell分配存储空间
*/
template <class Cell, const bool debug>
Array2D<Cell,debug>::Array2D(int xsize, int ysize)
{
//	assert(xsize>0);
//	assert(ysize>0);
	m_xsize=xsize;
	m_ysize=ysize;
	if (m_xsize>0 && m_ysize>0)
	{
		m_cells=new Cell*[m_xsize];
		for (int i=0; i<m_xsize; i++)
			m_cells[i]=new Cell[m_ysize];
	}
	else
	{
		m_xsize=m_ysize=0;
		m_cells=0;
	}
	if (debug)
	{
		std::cerr << __PRETTY_FUNCTION__ << std::endl;
		std::cerr << "m_xsize= " << m_xsize<< std::endl;
		std::cerr << "m_ysize= " << m_ysize<< std::endl;
	}
}

/*
@desc 重载运算符“=”，两个Array2D之间赋值
*/
template <class Cell, const bool debug>
Array2D<Cell,debug> & Array2D<Cell,debug>::operator=(const Array2D<Cell,debug> & g)
{
	if (debug || m_xsize!=g.m_xsize || m_ysize!=g.m_ysize)
	{
		for (int i=0; i<m_xsize; i++)
			delete [] m_cells[i];
		delete [] m_cells;
		m_xsize=g.m_xsize;
		m_ysize=g.m_ysize;
		m_cells=new Cell*[m_xsize];
		for (int i=0; i<m_xsize; i++)
			m_cells[i]=new Cell[m_ysize];
	}
	for (int x=0; x<m_xsize; x++)
		for (int y=0; y<m_ysize; y++)
			m_cells[x][y]=g.m_cells[x][y];
	
	if (debug)
	{
		std::cerr << __PRETTY_FUNCTION__ << std::endl;
		std::cerr << "m_xsize= " << m_xsize<< std::endl;
		std::cerr << "m_ysize= " << m_ysize<< std::endl;
	}
	return *this;
}

/*
@desc 构造函数
*/
template <class Cell, const bool debug>
Array2D<Cell,debug>::Array2D(const Array2D<Cell,debug> & g)
{
	m_xsize=g.m_xsize;
	m_ysize=g.m_ysize;
	m_cells=new Cell*[m_xsize];
	for (int x=0; x<m_xsize; x++)
	{
		m_cells[x]=new Cell[m_ysize];
		for (int y=0; y<m_ysize; y++)
			m_cells[x][y]=g.m_cells[x][y];
	}
	if (debug)
	{
		std::cerr << __PRETTY_FUNCTION__ << std::endl;
		std::cerr << "m_xsize= " << m_xsize<< std::endl;
		std::cerr << "m_ysize= " << m_ysize<< std::endl;
	}
}

/*
@desc 析构函数，释放申请的所有的内存空间
*/
template <class Cell, const bool debug>
Array2D<Cell,debug>::~Array2D()
{
  if (debug){
  	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "m_xsize= " << m_xsize<< std::endl;
	std::cerr << "m_ysize= " << m_ysize<< std::endl;
  }
  for (int i=0; i<m_xsize; i++)
  {
    delete [] m_cells[i];
    m_cells[i]=0;
  }
  delete [] m_cells;
  m_cells=0;
}

/*
@desc 清除函数，清除所有的数据
*/
template <class Cell, const bool debug>
void Array2D<Cell,debug>::clear()
{
  if (debug){
  	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "m_xsize= " << m_xsize<< std::endl;
	std::cerr << "m_ysize= " << m_ysize<< std::endl;
  }
  for (int i=0; i<m_xsize; i++)
  {
    delete [] m_cells[i];
    m_cells[i]=0;
  }
  delete [] m_cells;
  m_cells=0;
  m_xsize=0;
  m_ysize=0;
}

/*
@desc 重新设置地图大小
*/
template <class Cell, const bool debug>
void Array2D<Cell,debug>::resize(int xmin, int ymin, int xmax, int ymax)
{
	int xsize=xmax-xmin;
	int ysize=ymax-ymin;
	Cell ** newcells=new Cell *[xsize];
    for (int x=0; x<xsize; x++)
    {
		newcells[x]=new Cell[ysize];
	}
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

/*
@desc 判断一个点是否在地图内
*/
template <class Cell, const bool debug>
inline bool Array2D<Cell,debug>::isInside(int x, int y) const
{
	return x>=0 && y>=0 && x<m_xsize && y<m_ysize; 
}

/*
@desc 返回对应坐标的cell
*/
template <class Cell, const bool debug>
inline const Cell& Array2D<Cell,debug>::cell(int x, int y) const
{
	assert(isInside(x,y));
	return m_cells[x][y];
}

/*
@desc 返回对应坐标的Cell
*/
template <class Cell, const bool debug>
inline Cell& Array2D<Cell,debug>::cell(int x, int y)
{
	assert(isInside(x,y));
	return m_cells[x][y];
}

};

#endif

