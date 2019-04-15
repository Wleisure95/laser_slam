#ifndef SCANMATCHER_H
#define SCANMATCHER_H

#include "icp.h"
#include "smmap.h"
#include "eigen3/Eigen/Core"
#include "../include/gmapping/utils/macro_params.h"
#include "../include/gmapping/utils/stat.h"
#include <iostream>
#include "../include/gmapping/utils/gvalues.h"
#include "../include/gmapping/sensor/sensor_range/rangereading.h"

#define LASER_MAXBEAMS 2048

namespace GMapping {

class ScanMatcher{
public:
    typedef Covariance3 CovarianceMatrix;   //协方差

    ScanMatcher();
    ~ScanMatcher();


    /*
        icp优化求解位姿
        @param	pnew		求解的新的位姿
        @param  map			匹配的地图
        @param  P			老的位置
        @param	readings    激光数据
        */
    double icpOptimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;

    /*
        @desc 			根据旧的位姿和激光数据，通过跟地图匹配得到一个新的位姿
                        这个函数是gmapping实际使用的函数
        @param pnew 	得到的最优的位姿
        @param map  	地图
        @param p    	老的位置
        @pram readings  激光数据
        */
    double optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;

    //这个函数的功能跟上面的函数差不多 不过不是取最优的粒子，然后认为经过的路径服从高斯分布，因此最终的位姿是高斯分布的加权和
    //并且还能计算出来方差。
    double optimize(OrientedPoint& mean, CovarianceMatrix& cov, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;

    double   registerScan(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);

    /*设置激光参数 激光束、角度值、激光位置*/
    void setLaserParameters
    (unsigned int beams, double* angles, const OrientedPoint& lpose);

    /////////////////////////////////////////////////////

    void generateCorrelationMap(ScanMatcherMap& c_map,std::vector<RangeReading*>& base_scan);

    /////////////////////////////////////////////////////


    //设置匹配参数
    void setMatchingParameters
    (double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma=1, unsigned int likelihoodSkip=0 );

    void invalidateActiveArea();

    void computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);

    inline double icpStep(OrientedPoint & pret, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;

    inline double score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;

    inline unsigned int likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;

    double likelihood(double& lmax, OrientedPoint& mean, CovarianceMatrix& cov, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings);

    double likelihood(double& _lmax, OrientedPoint& _mean, CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& p, Gaussian3& odometry, const double* readings, double gain=180.);

    /*获取激光束的角度数据*/
    inline const double* laserAngles() const { return m_laserAngles; }

    /*获取激光束的数量*/
    inline unsigned int laserBeams() const { return m_laserBeams; }

    static const double nullLikelihood;
private:
    //ADD 07-04-2017
    double line_angle;

protected:
    //state of the matcher
    bool m_activeAreaComputed;

    /*机器人的m_laser储存这激光雷达(base_laser)在base_link坐标系中的坐标*/

    /**laser parameters*/
    unsigned int m_laserBeams;														//激光束的数量
    double       m_laserAngles[LASER_MAXBEAMS];										//各个激光束的角度
    //OrientedPoint m_laserPose;
    PARAM_SET_GET(OrientedPoint, laserPose, protected, public, public)				//激光的位置
    PARAM_SET_GET(double, laserMaxRange, protected, public, public)					//激光的最大测距范围
    /**scan_matcher parameters*/
    PARAM_SET_GET(double, usableRange, protected, public, public)					//使用的激光的最大范围
    PARAM_SET_GET(double, gaussianSigma, protected, public, public)
    PARAM_SET_GET(double, likelihoodSigma, protected, public, public)
    PARAM_SET_GET(int,    kernelSize, protected, public, public)
    PARAM_SET_GET(double, optAngularDelta, protected, public, public)				//优化时的角度增量
    PARAM_SET_GET(double, optLinearDelta, protected, public, public)				//优化时的长度增量
    PARAM_SET_GET(unsigned int, optRecursiveIterations, protected, public, public)	//优化时的迭代次数
    PARAM_SET_GET(unsigned int, likelihoodSkip, protected, public, public)
    PARAM_SET_GET(double, llsamplerange, protected, public, public)
    PARAM_SET_GET(double, llsamplestep, protected, public, public)
    PARAM_SET_GET(double, lasamplerange, protected, public, public)
    PARAM_SET_GET(double, lasamplestep, protected, public, public)
    PARAM_SET_GET(bool, generateMap, protected, public, public)
    PARAM_SET_GET(double, enlargeStep, protected, public, public)
    PARAM_SET_GET(double, fullnessThreshold, protected, public, public)				//被认为是占用的阈值
    PARAM_SET_GET(double, angularOdometryReliability, protected, public, public)	//里程计的角度可靠性
    PARAM_SET_GET(double, linearOdometryReliability, protected, public, public)		//里程计的长度可靠性
    PARAM_SET_GET(double, freeCellRatio, protected, public, public)					//free和occupany的阈值
    PARAM_SET_GET(unsigned int, initialBeamsSkip, protected, public, public)		//去掉初始的几个激光束的数量

    // allocate this large array only once
    IntPoint* m_linePoints;
};

/*
@desc	icpOptimize中的一步，目前还很不完善，不能实现正常的功能，还需要做修改。这个函数目前在gmapping中没有被使用
因为这里面理论上还需要调用icp.h文件中的优化函数来进行位姿的优化
@param	pret		新的位姿
@param  map			地图
@param  p			旧的位姿
@param  readings	激光数据
*/
inline double ScanMatcher::icpStep(OrientedPoint & pret, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const
{
    const double * angle=m_laserAngles+m_initialBeamsSkip;



    /*把激光雷达的坐标转换到世界坐标系下*/
    OrientedPoint lp=p;
    lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
    lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
    lp.theta+=m_laserPose.theta;

    unsigned int skip=0;
    double freeDelta=map.getDelta()*m_freeCellRatio;

    //用来存储对应的匹配的点云  主要是激光雷达的点云和地图栅格对应的点云
    std::list<PointPair> pairs;

    //这里主要是计算对应的点云对 计算要进行匹配的点云对
    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++)
    {
        skip++;
        skip=skip>m_likelihoodSkip?0:skip;
        if (*r>m_usableRange||*r==0.0) continue;
        if (skip) continue;

        /*被激光击中的点的地图坐标系*/
        Point phit=lp;
        phit.x+=*r*cos(lp.theta+*angle);
        phit.y+=*r*sin(lp.theta+*angle);
        IntPoint iphit=map.world2map(phit);

        //期望的空闲点的坐标
        Point pfree=lp;
        pfree.x+=(*r-map.getDelta()*freeDelta)*cos(lp.theta+*angle);
        pfree.y+=(*r-map.getDelta()*freeDelta)*sin(lp.theta+*angle);
        pfree=pfree-phit;
        IntPoint ipfree=map.world2map(pfree);

        /*在对应的kernerSize中搜索*/
        bool found=false;
        Point bestMu(0.,0.);
        Point bestCell(0.,0.);
        for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
            for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++)
            {
                IntPoint pr=iphit+IntPoint(xx,yy);
                IntPoint pf=pr+ipfree;

                //如果期望的击中是击中 期望的空闲是空闲 则说明合法
                const PointAccumulator& cell=map.cell(pr);
                const PointAccumulator& fcell=map.cell(pf);
                if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold)
                {
                    Point mu=phit-cell.mean();
                    if (!found)
                    {
                        bestMu=mu;
                        bestCell=cell.mean();
                        found=true;
                    }
                    else
                    {
                        if((mu*mu)<(bestMu*bestMu))
                        {
                            bestMu=mu;
                            bestCell=cell.mean();
                        }
                    }
                }
            }
        //如果找到则增加对应的匹配点云对
        if (found)
        {
            pairs.push_back(std::make_pair(phit, bestCell));
        }
    }


    //在上面计算完了对应的点云对之后，理论上这里应该调用icp.h里面的icp优化函数来进行位姿的优化
    OrientedPoint tc(0,0,0);

    //这里要考虑到底用线性还是非线性的方法来进行求解
    //线性方法收敛速度慢   但是耗时少
    //非线性方法收敛速度快 耗时多
    //求解得到两坨点云之间的转换关系
    //double error = icpLinearStep(tc,pairs);

    double error = 0;
    for(std::list<PointPair>::iterator it=pairs.begin(); it!=pairs.end(); it++)
    {
        Point delta(
                    it->first.x-it->second.x,
                    it->first.y-it->second.y);
        error+=delta*delta;
    }
    //    std::cerr<<"Sum error: "<<error<<std::endl;

    //这里计算出来的是phit 到bestCell的转换
    //也就是说bestCell = TC*Phit
    //error = icpNonlinearStep(tc,pairs);
    error = icpLinearStep(tc,pairs);
    //    std::cerr << "result(" << pairs.size() << ")=" << tc.x << " " << tc.y << " " << tc.theta << std::endl;
    //    std::cerr <<"error :"<<error<<std::endl;


    //    double tmp_x,tmp_y;
    //    tmp_x = ttot.x*cos(tc.theta) - ttot.y*sin(tc.theta) + tc.x;
    //    tmp_y = ttot.x*sin(tc.theta) + ttot.y*cos(tc.theta) + tc.y;

    //    ttot.x = tmp_x;
    //    ttot.y = tmp_y;
    //    ttot.theta+=tc.theta;
    //    ttot.theta=atan2(sin(ttot.theta), cos(ttot.theta));


    //result是在p这个坐标系下的 要转换到世界坐标系去。
    //Eigen::Matrix3d deltaT;
    //    std::cerr <<"before Score ICP:"<<score(map,p,readings)<<std::endl;
    double tp_x,tp_y;
    tp_x = p.x*cos(tc.theta) - p.y*sin(tc.theta) + tc.x;
    tp_y = p.x*sin(tc.theta) + p.y*cos(tc.theta) + tc.y;

    pret.x = tp_x;
    pret.y = tp_y;
    pret.theta=p.theta+tc.theta;
    pret.theta=atan2(sin(pret.theta), cos(pret.theta));
    //    std::cerr <<"after Score ICP:"<<score(map,pret,readings)<<std::endl;

    //进行更新
    //    pret.x=p.x+result.x;
    //	pret.y=p.y+result.y;
    //	pret.theta=p.theta+result.theta;
    //	pret.theta=atan2(sin(pret.theta), cos(pret.theta));

    return score(map, pret, readings);
}

/*
@desc 		根据地图、机器人位置、激光雷达数据，计算出一个得分：原理为likelihood_field_range_finder_model
这个函数被scanmatcher.cpp里面的optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings)
调用
@map  		对应的地图
@p    		机器人对应的初始位置
@readings	激光雷达数据
*/
inline double ScanMatcher::score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const
{
    double s=0;
    const double * angle=m_laserAngles+m_initialBeamsSkip;
    OrientedPoint lp=p;
    /*
    把激光雷达的坐标转换到世界坐标系
    先旋转到机器人坐标系，然后再转换到世界坐标系
    p表示base_link在map中的坐标
    m_laserPose 表示base_laser在base_link坐标系中的坐标
    */
    lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
    lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
    lp.theta+=m_laserPose.theta;

    /*
     * map.getDelta表示地图分辨率 m_freeCellRatio = sqrt(2)
     * 意思是如果激光hit了某个点 那么沿着激光方向的freeDelta距离的地方要是空闲才可以
    */
    unsigned int skip=0;
    double freeDelta=map.getDelta()*m_freeCellRatio;


    //枚举所有的激光束
    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++)
    {
        skip++;
        skip=skip>m_likelihoodSkip?0:skip;
        if (skip||*r>m_usableRange||*r==0.0) continue;

        /*被激光雷达击中的点 在地图坐标系中的坐标*/
        Point phit=lp;
        phit.x+=*r*cos(lp.theta+*angle);
        phit.y+=*r*sin(lp.theta+*angle);
        IntPoint iphit=map.world2map(phit);

        /*该束激光的最后一个空闲坐标，即紧贴hitCell的freeCell 原理为：假设phit是被激光击中的点，这样的话沿着激光方向的前面一个点必定的空闲的*/
        Point pfree=lp;
        //理论上来说 这个应该是一个bug。修改成下面的之后，改善不大。
        //		pfree.x+=(*r-map.getDelta()*freeDelta)*cos(lp.theta+*angle);
        //		pfree.y+=(*r-map.getDelta()*freeDelta)*sin(lp.theta+*angle);
        pfree.x+=(*r - freeDelta)*cos(lp.theta+*angle);
        pfree.y+=(*r - freeDelta)*sin(lp.theta+*angle);

        /*phit 和 freeCell的距离*/
        pfree=pfree-phit;
        IntPoint ipfree=map.world2map(pfree);

        /*在kernelSize大小的窗口中搜索出最优最可能被这个激光束击中的点 这个kernelSize在大小为1*/
        bool found=false;
        Point bestMu(0.,0.);
        for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
            for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++)
            {
                /*根据已开始的关系 搜索phit的时候，也计算出来pfree*/
                IntPoint pr=iphit+IntPoint(xx,yy);
                IntPoint pf=pr+ipfree;

                /*得到各自对应的Cell*/
                const PointAccumulator& cell=map.cell(pr);
                const PointAccumulator& fcell=map.cell(pf);
                /*
            (double)cell返回的是该cell被占用的概率
            这束激光要合法必须要满足cell是被占用的，而fcell是空闲的
            原理为：假设phit是被激光击中的点，这样的话沿着激光方向的前面一个点必定的空闲的
            */
                if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold)
                {
                    /*计算出被击中的点与对应的cell的currentScore距离*/
                    Point mu=phit-cell.mean();
                    if (!found)
                    {
                        bestMu=mu;
                        found=true;
                    }
                    else
                    {
                        bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
                    }
                }
            }
        /*socre的计算公式exp(-d^2 / sigma)) 这里的sigma表示方差 不是标准差*/
        if (found)
        {
            double tmp_score = exp(-1.0/m_gaussianSigma*bestMu*bestMu);
            s += tmp_score;
            //只在周围的9个栅格里面寻找，因此平方的意义不大。
            //s += tmp_score*tmp_score;
        }
    }
        return s;
}

/*
@desc 				根据地图、机器人位置、激光雷达数据，同时计算出一个得分和似然：原理为likelihood_field_range_finder_model
                    计算出来的似然即为粒子的权重
          这个函数被scanmatcher.cpp里面的optimize(OrientedPoint& _mean, ScanMatcher::CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings)
          调用

这个函数跟score是非常像的，不同的是这个函数除了计算的得分之外，还计算的似然likelihood
这个函数的计算出来的似然在gmapping中用来表示粒子的权重
@param s			得分
@param l			似然  粒子的权重
@param map  		对应的地图
@param p    		机器人对应的初始位置
@param readings		激光雷达数据
*/
inline unsigned int ScanMatcher::likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const
{
    using namespace std;
    l=0;
    s=0;
    const double * angle=m_laserAngles+m_initialBeamsSkip;

    /*
    把激光雷达的坐标转换到世界坐标系
    先旋转到机器人坐标系，然后再转换到世界坐标系
    */
    OrientedPoint lp=p;
    lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
    lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
    lp.theta+=m_laserPose.theta;

    //如果没有击中的时候的似然值 nullLikehood = -0.5
    double noHit=nullLikelihood/(m_likelihoodSigma);


    unsigned int skip=0;
    unsigned int c=0;
    double freeDelta=map.getDelta()*m_freeCellRatio;

    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++)
    {
        /*每隔m_likelihoodSkip个激光束 就跳过一个激光束*/
        skip++;
        skip=skip>m_likelihoodSkip?0:skip;
        if (*r>m_usableRange) continue;
        if (skip) continue;

        /*被激光击中的点*/
        Point phit=lp;
        phit.x+=*r*cos(lp.theta+*angle);
        phit.y+=*r*sin(lp.theta+*angle);
        IntPoint iphit=map.world2map(phit);

        Point pfree=lp;
        pfree.x+=(*r-freeDelta)*cos(lp.theta+*angle);
        pfree.y+=(*r-freeDelta)*sin(lp.theta+*angle);
        pfree=pfree-phit;
        IntPoint ipfree=map.world2map(pfree);

        /*在对应的kernerSize中搜索*/
        bool found=false;
        Point bestMu(0.,0.);
        for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
            for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++)
            {
                IntPoint pr=iphit+IntPoint(xx,yy);
                IntPoint pf=pr+ipfree;
                const PointAccumulator& cell=map.cell(pr);
                const PointAccumulator& fcell=map.cell(pf);

                /*如果cell(pr)被占用 而cell(pf)没有被占用 则说明找到了一个合法的点*/
                if (((double)cell )>m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold)
                {
                    Point mu=phit-cell.mean();
                    if (!found)
                    {
                        bestMu=mu;
                        found=true;
                    }
                    else
                    {
                        bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
                    }
                }
            }
        /*计算得分 得分是只计有用的激光束*/
        if (found)
        {
            s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
            c++;
        }

        /*计算似然 似然是计算所有的激光束 如果某一个激光束打中了空地 那也需要计算进去*/
        if (!skip)
        {
            //似然不是指数 似然只是指数的上标
            double f=(-1./m_likelihoodSigma)*(bestMu*bestMu);
            l+=(found)?f:noHit;
        }
    }
    return c;
}

};

#endif
