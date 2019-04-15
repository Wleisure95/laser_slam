#include <cstring>
#include <limits>
#include <list>
#include <iostream>
#include <omp.h>
#include "ros/ros.h"
#include "tf/LinearMath/Scalar.h"

#include "../include/gmapping/scanmatcher/scanmatcher.h"
#include "../include/gmapping/sensor/sensor_range/rangereading.h"
#include "gridlinetraversal.h"
//#define GENERATE_MAPS

namespace GMapping {

using namespace std;

const double ScanMatcher::nullLikelihood=-.5;

ScanMatcher::ScanMatcher(): m_laserPose(0,0,0)
{
    line_angle = 0.0;

	//m_laserAngles=0;
	m_laserBeams=0;
    m_optRecursiveIterations=9;
	m_activeAreaComputed=false;

	// This  are the dafault settings for a grid map of 5 cm
    // 5cm解析度的默认参数 定义scan-match的时候 这些参数会默认成这样。
    // 这个参数是计算似然位姿的时候使用的 实际的gmapping中没有用到。
	m_llsamplerange=0.01;
	m_llsamplestep=0.01;
	m_lasamplerange=0.005;
	m_lasamplestep=0.005;

    //地图进行拓展的大小
	m_enlargeStep=10.;

	m_fullnessThreshold=0.1;

    //指示里程计和陀螺仪是否可靠
    //如果可靠的话，那么进行score计算的时候，就需要对离里程计数据比较远的位姿增加惩罚
    //对于我们的应用来说 陀螺仪在短期内还是很可靠的。
	m_angularOdometryReliability=0.;
    m_linearOdometryReliability=0.;

    //理论上的离激光点的空闲距离 也就是说沿着激光束方向离激光点这么远距离的栅格一定是空闲的。
    m_freeCellRatio=sqrt(2.);

    //跳过一帧激光数据的开始几束激光
	m_initialBeamsSkip=0;	
/*	
	// This  are the dafault settings for a grid map of 10 cm
	m_llsamplerange=0.1;
	m_llsamplestep=0.1;
	m_lasamplerange=0.02;
	m_lasamplestep=0.01;
*/	
	// This  are the dafault settings for a grid map of 20/25 cm
/*
	m_llsamplerange=0.2;
	m_llsamplestep=0.1;
	m_lasamplerange=0.02;
	m_lasamplestep=0.01;
	m_generateMap=false;
*/

   m_linePoints = new IntPoint[20000];
}

ScanMatcher::~ScanMatcher()
{
	delete [] m_linePoints;
}

/**
 * @brief ScanMatcher::invalidateActiveArea
 * 每次调用computeActiveArea()之前，都必须要调用这个函数
 */
void ScanMatcher::invalidateActiveArea()
{
    m_activeAreaComputed=false;
}

/*
@desc 计算有效区域，通过激光雷达的数据计算出来哪个地图栅格应该要被更新了。(这里只是计算出来栅格的位置，然后插入地图中,并不对数据进行更新)
这里计算的有效区域的坐标都是patch坐标，不是cell坐标

注意！！！！！：：
这个函数在正常的进行SLAM算法的过程中，使用了m_generateMap = false。这个时候不会为了空闲区域分配内存。
当要生成可视化的地图的时候,m_generateMap = true。这个时候就会为空闲区域分配内存


@param map			地图
@param p			机器人位置
@param readings		激光雷达数据
*/
void ScanMatcher::computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const double* readings)
{
	/*已经计算过了，则没必要计算了*/
    if (m_activeAreaComputed)
        return;
	
	/*把激光雷达的位置转换到地图坐标系*/
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	IntPoint p0=map.world2map(lp);
	
	/*地图的范围*/
	Point min(map.map2world(0,0));
	Point max(map.map2world(map.getMapSizeX()-1,map.getMapSizeY()-1));
	       
	/*扩展地图的范围*/
	if (lp.x<min.x) min.x=lp.x;
	if (lp.y<min.y) min.y=lp.y;
	if (lp.x>max.x) max.x=lp.x;
	if (lp.y>max.y) max.y=lp.y;
	
	/*determine the size of the area*/
	/*根据激光数据扩展地图的范围*/
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++)
	{
		/*去除不合理的值*/
		if (*r>m_laserMaxRange||*r==0.0||isnan(*r)) continue;

        //根据设置截断gmapping的值
		double d=*r>m_usableRange?m_usableRange:*r;
		
		/*被激光击中的位置*/
		Point phit=lp;
		phit.x+=d*cos(lp.theta+*angle);
		phit.y+=d*sin(lp.theta+*angle);
		
		/*扩充范围*/
		if (phit.x<min.x) min.x=phit.x;
		if (phit.y<min.y) min.y=phit.y;
		if (phit.x>max.x) max.x=phit.x;
		if (phit.y>max.y) max.y=phit.y;
	}
	//min=min-Point(map.getDelta(),map.getDelta());
	//max=max+Point(map.getDelta(),map.getDelta());
	
	/*如果地图需要扩展*/
	if ( !map.isInside(min)	|| !map.isInside(max))
	{
		/*得到目前地图的大小*/
		Point lmin(map.map2world(0,0));
		Point lmax(map.map2world(map.getMapSizeX()-1,map.getMapSizeY()-1));
		//cerr << "CURRENT MAP " << lmin.x << " " << lmin.y << " " << lmax.x << " " << lmax.y << endl;
		//cerr << "BOUNDARY OVERRIDE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
		
		/*如果需要扩充，则把对应的维度扩展m_enlargeStep的大小*/
		min.x=( min.x >= lmin.x )? lmin.x: min.x-m_enlargeStep;
		max.x=( max.x <= lmax.x )? lmax.x: max.x+m_enlargeStep;
		min.y=( min.y >= lmin.y )? lmin.y: min.y-m_enlargeStep;
		max.y=( max.y <= lmax.y )? lmax.y: max.y+m_enlargeStep;
		map.resize(min.x, min.y, max.x, max.y);
		//cerr << "RESIZE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
	}
	
	/*地图的有效区域(地图坐标系)*/
	HierarchicalArray2D<PointAccumulator>::PointSet activeArea;

	/*allocate the active area*/
	angle=m_laserAngles+m_initialBeamsSkip;
	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++)
	{
		/*如果需要生成地图*/
		if (m_generateMap)
		{
            //排除错误的激光点
			double d=*r;
			if (d>m_laserMaxRange||d==0.0||isnan(d))
				continue;

			if (d>m_usableRange)
				d=m_usableRange;
			
			/*激光束的起点和终点*/
			Point phit=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
			IntPoint p0=map.world2map(lp);
			IntPoint p1=map.world2map(phit);
			
            /*bresenham算法来计算激光起点到终点要经过的路径*/
			GridLineTraversalLine line;
			line.points=m_linePoints;
			GridLineTraversal::gridLine(p0, p1, &line);
			
            /*更新地图 把画线算法计算出来的值都算进去*/
			for (int i=0; i<line.num_points-1; i++)
			{
				assert(map.isInside(m_linePoints[i]));
				activeArea.insert(map.storage().patchIndexes(m_linePoints[i]));
				assert(m_linePoints[i].x>=0 && m_linePoints[i].y>=0);
			}
            //如果激光距离小于使用的值 则需要把定点也算进去 说明这个值是好的。
            //同时如果d>=m_usableRange　那么说明这个值只用来进行标记空闲区域　不用来进行标记障碍物
			if (d<m_usableRange)
			{
				IntPoint cp=map.storage().patchIndexes(p1);
				assert(cp.x>=0 && cp.y>=0);
				activeArea.insert(cp);
			}
		} 
		else 
		{
			if (*r>m_laserMaxRange||*r>m_usableRange||*r==0.0||isnan(*r)) continue;
			Point phit=lp;
			phit.x+=*r*cos(lp.theta+*angle);
			phit.y+=*r*sin(lp.theta+*angle);
			IntPoint p1=map.world2map(phit);
			assert(p1.x>=0 && p1.y>=0);
			IntPoint cp=map.storage().patchIndexes(p1);
			assert(cp.x>=0 && cp.y>=0);
			activeArea.insert(cp);
		}
	}
	
	map.storage().setActiveArea(activeArea, true);
	m_activeAreaComputed=true;
}

/*
@desc		已知位置的覆盖栅格地图算法(使用的模型为Counting Model)
            Counting Model：某一个点被覆盖的概率为被占用的次数与被访问的次数的比值
@map		对应的地图
@p			机器人的位姿      机器人的位姿  不是激光雷达的位姿
@reading    激光雷达的数据
*/
double ScanMatcher::registerScan(ScanMatcherMap& map, const OrientedPoint& p, const double* readings)
{
    if (!m_activeAreaComputed)
        computeActiveArea(map, p, readings);

    //每次registerScan的时候都进行计算
    computeActiveArea(map, p, readings);
		
	//this operation replicates the cells that will be changed in the registration operation
    //为activeArea里面的没有分配内存的区域分配内存
	map.storage().allocActiveArea();
	
	/*把激光雷达的位置转换到地图坐标系*/
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	IntPoint p0=map.world2map(lp);
	
	
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	/*esum表示总体的熵的增加或者减少*/
	double esum=0;
	for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++)
	{
		if (m_generateMap)
		{
			/*去除非法的激光束*/
			double d=*r;
			if (d>m_laserMaxRange||d==0.0||isnan(d))
				continue;
			if (d>m_usableRange)
				d=m_usableRange;
			
			/*被该激光束击中的点的地图坐标*/
			Point phit=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
			IntPoint p1=map.world2map(phit);
			//IntPoint linePoints[20000] ;
			
			/*bresenham画线算法来计算 激光位置和被激光击中的位置之间的空闲位置*/
			GridLineTraversalLine line;
			line.points=m_linePoints;
			GridLineTraversal::gridLine(p0, p1, &line);
			
			/*更新空闲位置*/
			for (int i=0; i<line.num_points-1; i++)
			{
				PointAccumulator& cell=map.cell(line.points[i]);
				/*更新前的熵的负数*/
				double e=-cell.entropy();       
				cell.update(false, Point(0,0));
				/*加上更新后的熵，即更新后的熵减去更新前的熵，求得这次更新对于熵的变化*/
				e+=cell.entropy();
				esum+=e;
			}
			
            /*更新被击中的位置　只有小于m_usableRange的栅格来用来标记障碍物*/
			if (d<m_usableRange)
			{
				double e=-map.cell(p1).entropy();
				map.cell(p1).update(true, phit);
				e+=map.cell(p1).entropy();
				esum+=e;
			}
		} 
		else 
		{
			if (*r>m_laserMaxRange||*r>m_usableRange||*r==0.0||isnan(*r)) continue;
			/*被击中的点的地图坐标*/
			Point phit=lp;
			phit.x+=*r*cos(lp.theta+*angle);
			phit.y+=*r*sin(lp.theta+*angle);
			IntPoint p1=map.world2map(phit);
			assert(p1.x>=0 && p1.y>=0);
			
			/*更新对应的cell的值*/
			map.cell(p1).update(true,phit);
		}
	}
	return esum;
}

/*
@desc	根据地图、激光数据、位姿用icp迭代求解出一个最优的位姿来
理论上这种优化方式应该要比类似梯度下降方法的效果要好。
但实际上比那种的效果差，不知道为什么
@param	pnew		新的最优位置
@param  map			地图
@param	init		初始位置
@param  readings	激光数据
*/

double ScanMatcher::icpOptimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings) const
{
	double currentScore;
    double sc=score(map, init, readings);
	OrientedPoint start=init;
    OrientedPoint icp_pose = init;
	pnew=init;
	
    //std::cerr<<"icpOptimiza Start"<<std::endl;
	/*一直优化到性能没有提升了为止*/
	int iterations=0;
    do{
        pnew = icp_pose;

		currentScore=sc;
        sc=icpStep(icp_pose, map, start, readings);
        start=icp_pose;

        iterations++;
	} while (sc>currentScore);
	
    //cerr << "i="<< iterations << endl;
    //std::cerr<<"icpOptimiza End"<<std::endl;

	return currentScore;
}


/*
/ zq commit
@desc	根据地图、激光数据、位姿迭代求解一个最优的新的位姿出来
这个函数是真正被调用来进行scan-match的函数
@param	pnew		新的最优位置
@param  map			地图
@param	init		初始位置
@param  readings	激光数据
*/
double ScanMatcher::optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings) const
{
    double bestScore=-1;
    /*计算当前位置的得分*/
    OrientedPoint currentPose=init;
    double currentScore=score(map, currentPose, readings);
	
    /*所有时的步进增量*/
    double adelta=m_optAngularDelta, ldelta=m_optLinearDelta;
	
    /*精确搜索的次数*/
    unsigned int refinement=0;
	
    /*搜索的方向*/
    enum Move{Front, Back, Left, Right, TurnLeft, TurnRight, Done};
    //enum Move{Front, Back, Left, Right, TurnLeft, TurnRight, Done};
    int c_iterations=0;
    do
    {
        /*如果这一次(currentScore)算出来比上一次(bestScore)差，则有可能是走太多了，要减少搜索步长 这个策略跟LM有点像*/
        if (bestScore>=currentScore)
        {
            refinement++;
            adelta*=.5;
            ldelta*=.5;
        }
        bestScore=currentScore;
        OrientedPoint bestLocalPose=currentPose;
        OrientedPoint localPose=currentPose;

        /*把8个方向都搜索一次  得到这8个方向里面最好的一个位姿和对应的得分*/
        Move move=Front;
        do
        {
            localPose=currentPose;
            switch(move)
            {
                case Front:
                    localPose.x+=ldelta;
                    move=Back;
                    break;
                case Back:
                    localPose.x-=ldelta;
                    move=Left;
                    break;
                case Left:
                    localPose.y-=ldelta;
                    move=Right;
                    break;
                case Right:
                    localPose.y+=ldelta;
                    move=TurnLeft;
                    break;
                case TurnLeft:
                    localPose.theta+=adelta;
                    move=TurnRight;
                    break;
                case TurnRight:
                    localPose.theta-=adelta;
                    move=Done;
                    break;
                default:;
            }
			
            //计算当前的位姿和初始位姿的区别 区别越大增益越小
            double odo_gain=1;

            //计算当前位姿的角度和初始角度的区别 如果里程计比较可靠的话
            //那么进行匹配的时候就需要对离初始位姿比较远的位姿施加惩罚
            if (m_angularOdometryReliability>0.)
            {
                double dth=init.theta-localPose.theta; 	dth=atan2(sin(dth), cos(dth)); 	dth*=dth;
                odo_gain*=exp(-m_angularOdometryReliability*dth);
            }
            //计算线性距离的区别 线性距离也是一样
            if (m_linearOdometryReliability>0.)
            {
                double dx=init.x-localPose.x;
                double dy=init.y-localPose.y;
                double drho=dx*dx+dy*dy;
                odo_gain*=exp(-m_linearOdometryReliability*drho);
            }
            /*计算得分=增益*score*/
            double localScore=odo_gain*score(map, localPose, readings);
			
            /*如果得分更好，则更新*/
            if (localScore>currentScore)
            {
                currentScore=localScore;
                bestLocalPose=localPose;
            }
            c_iterations++;
        } while(move!=Done);
		
        /* 把当前位置设置为目前最优的位置  如果8个值都被差了的话，那么这个值不会更新*/
        currentPose=bestLocalPose;
    }while (currentScore>bestScore || refinement<m_optRecursiveIterations);
	
    /*返回最优位置和得分*/
    pnew=currentPose;
    return bestScore;
}


//表示一个位姿和这个位姿的似然+得分
struct ScoredMove
{
	OrientedPoint pose;
	double score;
	double likelihood;
};

//用来存储在scan-match的过程中 机器人的优化的路径的节点
//主要在下面的optimize()函数里面被调用
typedef std::list<ScoredMove> ScoredMoveList;

/*
@desc	根据地图、激光数据、位姿迭代求解一个最优位姿来。
这个函数与上面的函数不同的是，这个函数求出来的位姿不是一个特定的最优的cell，
而是假设在scan-match的过程中经过的所有的点服从高斯分布，然后根据经过的所有位姿的似然来求解得到一个加权位姿和位姿的方差。
这里认为机器人在scan-match过程中经过的路径点合起来是服从高斯分布的，因此机器人的位姿和方差最终由所有经过的点的加权和来计算得到

理论上来说这是一个更合理的选择。因为GMapping的论文里面也是选择了这样的方法


@param	pnew		新的最优位置
@param  map			地图
@param	init		初始位置
@param  readings	激光数据
*/
double ScanMatcher::optimize(OrientedPoint& _mean, ScanMatcher::CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings) const
{
	ScoredMoveList moveList;
	double bestScore=-1;
	
	/*计算当前位置的得分score和似然likehood*/
	OrientedPoint currentPose=init;
	ScoredMove sm={currentPose,0,0};
	unsigned int matched=likelihoodAndScore(sm.score, sm.likelihood, map, currentPose, readings);
	double currentScore=sm.score;
	moveList.push_back(sm);
	
	/*角度和线性递增*/
	double adelta=m_optAngularDelta, ldelta=m_optLinearDelta;
	
	unsigned int refinement=0;
	int count=0;
	
	/*8个方向迭代优化，找到一个最优的位置*/
	enum Move{Front, Back, Left, Right, TurnLeft, TurnRight, Done};
	do{
        if (bestScore>=currentScore)
        {
			refinement++;
			adelta*=.5;
			ldelta*=.5;
		}
		bestScore=currentScore;
		OrientedPoint bestLocalPose=currentPose;
		OrientedPoint localPose=currentPose;

		Move move=Front;
		do {
			localPose=currentPose;
            switch(move)
            {
                case Front:
					localPose.x+=ldelta;
					move=Back;
					break;
				case Back:
					localPose.x-=ldelta;
					move=Left;
					break;
				case Left:
					localPose.y-=ldelta;
					move=Right;
					break;
				case Right:
					localPose.y+=ldelta;
					move=TurnLeft;
					break;
				case TurnLeft:
					localPose.theta+=adelta;
					move=TurnRight;
					break;
				case TurnRight:
					localPose.theta-=adelta;
					move=Done;
					break;
				default:;
			}
			double localScore, localLikelihood;
			
            //计算似然的增益

            //如果里程计很可靠的话，那么现在的位姿如果离里程计位姿比较远的话，就需要加入惩罚
			double odo_gain=1;
            if (m_angularOdometryReliability>0.)
            {
                double dth=init.theta-localPose.theta;
                dth=atan2(sin(dth), cos(dth));
                dth*=dth;
				odo_gain*=exp(-m_angularOdometryReliability*dth);
			}
            if (m_linearOdometryReliability>0.)
            {
				double dx=init.x-localPose.x;
				double dy=init.y-localPose.y;
				double drho=dx*dx+dy*dy;
				odo_gain*=exp(-m_linearOdometryReliability*drho);
			}

            //这个分数根本就没有用。。。不知道搞什么鬼
			localScore=odo_gain*score(map, localPose, readings);
			//update the score
			count++;
			
            /*重新计算score 和 localLikelihood 也就是说上面计算得到的score是没有用的*/
            //这里面调用的是scanmatcher.h这个文件家里面的likelihoodAndScore函数
			matched=likelihoodAndScore(localScore, localLikelihood, map, localPose, readings);

            //如果大于当前得分 则需要进行更新
            if (localScore>currentScore)
            {
				currentScore=localScore;
				bestLocalPose=localPose;
			}
			
            //记录下来所有经过的点 以及对应的似然
			sm.score=localScore;
			sm.likelihood=localLikelihood;//+log(odo_gain);
			sm.pose=localPose;
			moveList.push_back(sm);

			//update the move list
		} while(move!=Done);
		currentPose=bestLocalPose;
	}while (currentScore>bestScore || refinement<m_optRecursiveIterations);
	
	//normalize the likelihood
	/*归一化似然*/
	double lmin=1e9;
	double lmax=-1e9;
	/*求取最大最小似然*/
    for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++)
    {
		lmin=it->likelihood<lmin?it->likelihood:lmin;
		lmax=it->likelihood>lmax?it->likelihood:lmax;
	}
	
    /*执行归一化操作 把似然通过指数函数计算成为得分 也就是概率*/
    for (ScoredMoveList::iterator it=moveList.begin(); it!=moveList.end(); it++)
    {
		it->likelihood=exp(it->likelihood-lmax);
	}
	
	//compute the mean
	/*计算均值 和 方差*/
	OrientedPoint mean(0,0,0);
	double lacc=0;
	
	/*计算均值*/
    for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++)
    {
		mean=mean+it->pose*it->likelihood;
		lacc+=it->likelihood;
	}
	mean=mean*(1./lacc);
	
	/*计算方差*/
	CovarianceMatrix cov={0.,0.,0.,0.,0.,0.};
    for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++)
    {
		OrientedPoint delta=it->pose-mean;
		delta.theta=atan2(sin(delta.theta), cos(delta.theta));

		cov.xx+=delta.x*delta.x*it->likelihood;
		cov.yy+=delta.y*delta.y*it->likelihood;
		cov.tt+=delta.theta*delta.theta*it->likelihood;
		cov.xy+=delta.x*delta.y*it->likelihood;
		cov.xt+=delta.x*delta.theta*it->likelihood;
		cov.yt+=delta.y*delta.theta*it->likelihood;
	}
	cov.xx/=lacc, cov.xy/=lacc, cov.xt/=lacc, cov.yy/=lacc, cov.yt/=lacc, cov.tt/=lacc;
	
	_mean=currentPose;
	_cov=cov;
	return bestScore;
}

	
/*
计算某一个机器人的位置的似然，这个似然是在机器人位置的一个范围内(-mllsamplerange~mllsamplerange  -mllsamplerange~mllsamplerange -mlasamplerange m_lasamplerange)的似然
并且通过这个范围内的似然分布，来求出机器人的位置的期望值和方差

这个函数的真实意图应该是计算机器人位姿p的似然值。
这个也是Cyrill Stachniss在论文里面提出的方法。
这个方法应该是和上面的optimiza(_mean,_cov,...)方法配套使用的。
用optimiza来计算位姿 然后用这个函数来计算似然(或者说对应的位姿的权重)
可以认为这个函数的返回值就是对应的粒子的权重

这个函数的意思是：
假设机器人在p附近的(-mllsamplerange~mllsamplerange  -mllsamplerange~mllsamplerange -mlasamplerange m_lasamplerange)窗口是服从高斯分布的。
求机器人的真实位姿以及机器人的方差
*/
double ScanMatcher::likelihood
    (double& _lmax, OrientedPoint& _mean, CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings)
{
	ScoredMoveList moveList;
	
	/*计算这个区域内的每个点的似然 和 得分*/
	for (double xx=-m_llsamplerange; xx<=m_llsamplerange; xx+=m_llsamplestep)
	for (double yy=-m_llsamplerange; yy<=m_llsamplerange; yy+=m_llsamplestep)
	for (double tt=-m_lasamplerange; tt<=m_lasamplerange; tt+=m_lasamplestep)
	{
		
		OrientedPoint rp=p;
		rp.x+=xx;
		rp.y+=yy;
		rp.theta+=tt;
		
		ScoredMove sm;
		sm.pose=rp;
		
		likelihoodAndScore(sm.score, sm.likelihood, map, rp, readings);
		moveList.push_back(sm);
	}
	
	//normalize the likelihood
    /*归一化似然*/
    double lmax=-1e9; //最大的似然值
    double lcum=0;    //累计的似然值
	
    /*求取最大的似然值*/
    for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++)
    {
		lmax=it->likelihood>lmax?it->likelihood:lmax;
	}
	
	/*计算lcum 和 每个点的概率p(x)    归一化似然*/
    for (ScoredMoveList::iterator it=moveList.begin(); it!=moveList.end(); it++)
    {
		//it->likelihood=exp(it->likelihood-lmax);
		lcum+=exp(it->likelihood-lmax);
		it->likelihood=exp(it->likelihood-lmax);
		//cout << "l=" << it->likelihood << endl;
	}
	//到了这里这个区域内的似然都已经归一化了
	
	/*计算均值  机器人位置的均值  E(x) = \sum_{x*p(x)}*/
	OrientedPoint mean(0,0,0);
	double s=0,c=0;
    for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++)
    {
		mean=mean+it->pose*it->likelihood;
		s+=it->likelihood*sin(it->pose.theta);
		c+=it->likelihood*cos(it->pose.theta);
	}
	mean=mean*(1./lcum);

    //计算机器人的位姿
    s/=lcum;
	c/=lcum;
	mean.theta=atan2(s,c);
	
	/*计算方差 机器人位置的方差*/
	CovarianceMatrix cov={0.,0.,0.,0.,0.,0.};
	for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++)
	{
		
		/*计算每个点和mean的距离差和角度差*/
		OrientedPoint delta=it->pose-mean;
		delta.theta=atan2(sin(delta.theta), cos(delta.theta));
		
		/*计算协防差矩阵*/
		cov.xx+=delta.x*delta.x*it->likelihood;
		cov.yy+=delta.y*delta.y*it->likelihood;
		cov.tt+=delta.theta*delta.theta*it->likelihood;
		cov.xy+=delta.x*delta.y*it->likelihood;
		cov.xt+=delta.x*delta.theta*it->likelihood;
		cov.yt+=delta.y*delta.theta*it->likelihood;
	}
    //协方差矩阵进行归一化
	cov.xx/=lcum, cov.xy/=lcum, cov.xt/=lcum, cov.yy/=lcum, cov.yt/=lcum, cov.tt/=lcum;
	
    //把值传递出去
	_mean=mean;
	_cov=cov;
	_lmax=lmax;

    //返回极大似然值和累计似然值 意思就是不单单是取似然值最大的那个 也不是取累计似然值最大的那个
	return log(lcum)+lmax;
}


/*跟上面的函数基本上是一个功能，不同的是这个函数考虑了里程计的似然*/
double ScanMatcher::likelihood
	(double& _lmax, OrientedPoint& _mean, CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& p,
    Gaussian3& odometry, const double* readings, double gain)
{
	ScoredMoveList moveList;
	
	
	for (double xx=-m_llsamplerange; xx<=m_llsamplerange; xx+=m_llsamplestep)
	for (double yy=-m_llsamplerange; yy<=m_llsamplerange; yy+=m_llsamplestep)
	for (double tt=-m_lasamplerange; tt<=m_lasamplerange; tt+=m_lasamplestep)
	{
		
		OrientedPoint rp=p;
		rp.x+=xx;
		rp.y+=yy;
		rp.theta+=tt;
		
		ScoredMove sm;
		sm.pose=rp;
		
		
		likelihoodAndScore(sm.score, sm.likelihood, map, rp, readings);
		sm.likelihood+=odometry.eval(rp)/gain;
		assert(!isnan(sm.likelihood));
		moveList.push_back(sm);
	}
	
   //normalize the likelihood	归一化似然
  double lmax=-std::numeric_limits<double>::max();
	double lcum=0;
	for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++)
	{
		lmax=it->likelihood>lmax?it->likelihood:lmax;
	}
	for (ScoredMoveList::iterator it=moveList.begin(); it!=moveList.end(); it++)
	{
		//it->likelihood=exp(it->likelihood-lmax);
		lcum+=exp(it->likelihood-lmax);
		it->likelihood=exp(it->likelihood-lmax);
		//cout << "l=" << it->likelihood << endl;
	}
	//似然归一化完毕
	
	//计算均值
	OrientedPoint mean(0,0,0);
	double s=0,c=0;
    for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++)
    {
		mean=mean+it->pose*it->likelihood;
		s+=it->likelihood*sin(it->pose.theta);
		c+=it->likelihood*cos(it->pose.theta);
	}
	mean=mean*(1./lcum);
	s/=lcum;
	c/=lcum;
	mean.theta=atan2(s,c);
	
	//计算方差
	CovarianceMatrix cov={0.,0.,0.,0.,0.,0.};
	for (ScoredMoveList::const_iterator it=moveList.begin(); it!=moveList.end(); it++){
		OrientedPoint delta=it->pose-mean;
		delta.theta=atan2(sin(delta.theta), cos(delta.theta));
		cov.xx+=delta.x*delta.x*it->likelihood;
		cov.yy+=delta.y*delta.y*it->likelihood;
		cov.tt+=delta.theta*delta.theta*it->likelihood;
		cov.xy+=delta.x*delta.y*it->likelihood;
		cov.xt+=delta.x*delta.theta*it->likelihood;
		cov.yt+=delta.y*delta.theta*it->likelihood;
	}
	cov.xx/=lcum, cov.xy/=lcum, cov.xt/=lcum, cov.yy/=lcum, cov.yt/=lcum, cov.tt/=lcum;
	
	_mean=mean;
	_cov=cov;
	_lmax=lmax;
	double v=log(lcum)+lmax;
	assert(!isnan(v));
	return v;
}

/*
@desc			设置激光的参数
@param beams	激光束的数量
@param angles   激光束对应的角度值
@param lpose    激光的位置
*/
void ScanMatcher::setLaserParameters
    (unsigned int beams, double* angles, const OrientedPoint& lpose)
{
    assert(beams<LASER_MAXBEAMS);
    m_laserPose=lpose;
    m_laserBeams=beams;
    //m_laserAngles=new double[beams];
    memcpy(m_laserAngles, angles, sizeof(double)*m_laserBeams);
}


/*设置匹配的参数*/
void ScanMatcher::setMatchingParameters
    (double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations,  double likelihoodSigma, unsigned int likelihoodSkip)
{
	m_usableRange=urange;						//传感器的使用范围
	m_laserMaxRange=range;						//传感器的最大范围
	m_kernelSize=kernsize;						//kernsize主要用在计算score时搜索框的大小
	m_optLinearDelta=lopt;						//优化时的线性步长
	m_optAngularDelta=aopt;						//优化时的角度步长
	m_optRecursiveIterations=iterations;		//优化时的迭代次数
	m_gaussianSigma=sigma;						//计算socre时的方差
	m_likelihoodSigma=likelihoodSigma;			//计算似然时的方差	
	m_likelihoodSkip=likelihoodSkip;			//计算似然时，跳过的激光束
}

};

