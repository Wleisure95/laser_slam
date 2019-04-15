#include <string>
#include <deque>
#include <list>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>
#include "../include/gmapping/utils/stat.h"
#include "../include/gmapping/gridfastslam/gridslamprocessor.h"
#include "../include/gmapping/utils/point.h"
#include <omp.h>

//#define MAP_CONSISTENCY_CHECK
//#define GENERATE_TRAJECTORIES

namespace GMapping {

const double m_distanceThresholdCheck = 5;
 
using namespace std;

  GridSlamProcessor::GridSlamProcessor(): m_infoStream(cout)
  {  
    period_ = 5.0;
    m_obsSigmaGain=1;
    m_resampleThreshold=0.5;
    m_minimumScore=0.;
  }
  
  GridSlamProcessor::GridSlamProcessor(const GridSlamProcessor& gsp) 
    :last_update_time_(0.0), m_particles(gsp.m_particles), m_infoStream(cout)
  {
    
    period_ = 5.0;

    m_obsSigmaGain=gsp.m_obsSigmaGain;
    m_resampleThreshold=gsp.m_resampleThreshold;
    m_minimumScore=gsp.m_minimumScore;
    
    m_beams=gsp.m_beams;
    m_indexes=gsp.m_indexes;
    m_motionModel=gsp.m_motionModel;
    m_resampleThreshold=gsp.m_resampleThreshold;
    m_matcher=gsp.m_matcher;
    
    m_count=gsp.m_count;
    m_readingCount=gsp.m_readingCount;
    m_lastPartPose=gsp.m_lastPartPose;
    m_pose=gsp.m_pose;
    m_odoPose=gsp.m_odoPose;
    m_linearDistance=gsp.m_linearDistance;
    m_angularDistance=gsp.m_angularDistance;
    m_neff=gsp.m_neff;
	
    cerr << "FILTER COPY CONSTRUCTOR" << endl;
    cerr << "m_odoPose=" << m_odoPose.x << " " <<m_odoPose.y << " " << m_odoPose.theta << endl;
    cerr << "m_lastPartPose=" << m_lastPartPose.x << " " <<m_lastPartPose.y << " " << m_lastPartPose.theta << endl;
    cerr << "m_linearDistance=" << m_linearDistance << endl;
    cerr << "m_angularDistance=" << m_linearDistance << endl;
    
		
    m_xmin=gsp.m_xmin;
    m_ymin=gsp.m_ymin;
    m_xmax=gsp.m_xmax;
    m_ymax=gsp.m_ymax;
    m_delta=gsp.m_delta;
    
    m_regScore=gsp.m_regScore;
    m_critScore=gsp.m_critScore;
    m_maxMove=gsp.m_maxMove;
    
    m_linearThresholdDistance=gsp.m_linearThresholdDistance;
    m_angularThresholdDistance=gsp.m_angularThresholdDistance;
    m_obsSigmaGain=gsp.m_obsSigmaGain;
    
#ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ <<  ": trajectories copy.... ";
#endif
    TNodeVector v=gsp.getTrajectories();
    for (unsigned int i=0; i<v.size(); i++){
		m_particles[i].node=v[i];
    }
#ifdef MAP_CONSISTENCY_CHECK
    cerr <<  "end" << endl;
#endif


    cerr  << "Tree: normalizing, resetting and propagating weights within copy construction/cloneing ..." ;
    updateTreeWeights(false);
    cerr  << ".done!" <<endl;
  }
  
  GridSlamProcessor::GridSlamProcessor(std::ostream& infoS): m_infoStream(infoS)
  {
    period_ = 5.0;
    m_obsSigmaGain=1;
    m_resampleThreshold=0.5;
    m_minimumScore=0.;
	
  }

  GridSlamProcessor* GridSlamProcessor::clone() const
  {
# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing preclone_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
	for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	  const ScanMatcherMap& m1(it->map);
	  const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
 	  for (int x=0; x<h1.getXSize(); x++){
	    for (int y=0; y<h1.getYSize(); y++){
	      const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	      if (a1.m_reference){
		PointerMap::iterator f=pmap.find(a1.m_reference);
		if (f==pmap.end())
		  pmap.insert(make_pair(a1.m_reference, 1));
		else
		  f->second++;
	      }
	    }
	  }
	}
	cerr << __PRETTY_FUNCTION__ <<  ": Number of allocated chunks" << pmap.size() << endl;
	for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
	  assert(it->first->shares==(unsigned int)it->second);

	cerr << __PRETTY_FUNCTION__ <<  ": SUCCESS, the error is somewhere else" << endl;
# endif
	GridSlamProcessor* cloned=new GridSlamProcessor(*this);
	
# ifdef MAP_CONSISTENCY_CHECK
	cerr << __PRETTY_FUNCTION__ <<  ": trajectories end" << endl;
	cerr << __PRETTY_FUNCTION__ << ": performing afterclone_fit_test" << endl;
	ParticleVector::const_iterator jt=cloned->m_particles.begin();
	for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
	  const ScanMatcherMap& m1(it->map);
	  const ScanMatcherMap& m2(jt->map);
	  const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
	  const HierarchicalArray2D<PointAccumulator>& h2(m2.storage());
	  jt++;
 	  for (int x=0; x<h1.getXSize(); x++){
	    for (int y=0; y<h1.getYSize(); y++){
	      const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	      const autoptr< Array2D<PointAccumulator> >& a2(h2.m_cells[x][y]);
	      assert(a1.m_reference==a2.m_reference);
	      assert((!a1.m_reference) || !(a1.m_reference->shares%2));
	    }
	  }
	}
	cerr << __PRETTY_FUNCTION__ <<  ": SUCCESS, the error is somewhere else" << endl;
# endif
	return cloned;
}
  
  GridSlamProcessor::~GridSlamProcessor()
  {
    cerr << __PRETTY_FUNCTION__ << ": Start" << endl;
    cerr << __PRETTY_FUNCTION__ << ": Deleting tree" << endl;
    for (std::vector<Particle>::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
#ifdef TREE_CONSISTENCY_CHECK		
      TNode* node=it->node;
      while(node)
	node=node->parent;
      cerr << "@" << endl;
#endif
      if (it->node)
	delete it->node;
      //cout << "l=" << it->weight<< endl;
    }
    
# ifdef MAP_CONSISTENCY_CHECK
    cerr << __PRETTY_FUNCTION__ << ": performing predestruction_fit_test" << endl;
    typedef std::map<autoptr< Array2D<PointAccumulator> >::reference* const, int> PointerMap;
    PointerMap pmap;
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
      const ScanMatcherMap& m1(it->map);
      const HierarchicalArray2D<PointAccumulator>& h1(m1.storage());
      for (int x=0; x<h1.getXSize(); x++){
	for (int y=0; y<h1.getYSize(); y++){
	  const autoptr< Array2D<PointAccumulator> >& a1(h1.m_cells[x][y]);
	  if (a1.m_reference){
	    PointerMap::iterator f=pmap.find(a1.m_reference);
	    if (f==pmap.end())
	      pmap.insert(make_pair(a1.m_reference, 1));
	    else
	      f->second++;
	  }
	}
      }
    }
    cerr << __PRETTY_FUNCTION__ << ": Number of allocated chunks" << pmap.size() << endl;
    for(PointerMap::const_iterator it=pmap.begin(); it!=pmap.end(); it++)
      assert(it->first->shares>=(unsigned int)it->second);
    cerr << __PRETTY_FUNCTION__ << ": SUCCESS, the error is somewhere else" << endl;
# endif
  }



  /*
  @desc 设置扫描匹配的参数 各个参数的定义 可以看m_matcher.setMatchingParameters函数里面
  urange
  range
  sigma
  kersize
  lopt
  aopt
  iterations
  */
  void GridSlamProcessor::setMatchingParameters (double urange, double range, double sigma, int kernsize, double lopt, double aopt, 
                         int iterations, double likelihoodSigma, double likelihoodGain, unsigned int likelihoodSkip)
  {
    m_obsSigmaGain=likelihoodGain;
    m_matcher.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip);
  }
  
/*
@desc 设置里程计运动模型的参数
@srr  线性运动造成的线性误差的方差
@srt  线性运动造成的角度误差的方差
@str  旋转运动造成的线性误差的方差
@stt  旋转运动造成的角度误差的方差
*/  
void GridSlamProcessor::setMotionModelParameters
(double srr, double srt, double str, double stt)
{
  m_motionModel.srr=srr;
  m_motionModel.srt=srt;
  m_motionModel.str=str;
  m_motionModel.stt=stt;	
}
  
  /*
  @desc 				设置机器人更新滤波器的阈值
  @linear  				走过的线性距离
  @angular 				走过的角度距离
  @resampleThreshold	经过的时间
  */
  void GridSlamProcessor::setUpdateDistances(double linear, double angular, double resampleThreshold)
  {
    m_linearThresholdDistance=linear; 
    m_angularThresholdDistance=angular;
    m_resampleThreshold=resampleThreshold;	
  }
  
  //HERE STARTS THE BEEF

//  不知道为什么 如果加上了低分辨率的地图 那么这个构造函数就在编译的时候就会报错。
//  只有改成下面那样的构造函数才行。
//  GridSlamProcessor::Particle::Particle(const ScanMatcherMap& m):
//    map(m),pose(0,0,0), weight(0), weightSum(0), gweight(0), previousIndex(0)
//  {
//    node=0;
//  }
  /**
   * @brief GridSlamProcessor::Particle::Particle
   * 自己定义的用低分辨率和高分辨率一起初始化的构造函数
   * @param m
   * @param lowMap
   */
  GridSlamProcessor::Particle::Particle(const ScanMatcherMap& m,const ScanMatcherMap& lowMap):
    map(m), lowResolutionMap(lowMap),pose(0,0,0), weight(0), weightSum(0), gweight(0), previousIndex(0)
  {
    node=0;
  }
  
  
  void GridSlamProcessor::setSensorMap(const SensorMap& smap)
  {
    
    /*
      Construct the angle table for the sensor
      
      FIXME For now detect the readings of only the front laser, and assume its pose is in the center of the robot 
    */
    
    SensorMap::const_iterator laser_it=smap.find(std::string("FLASER"));
    if (laser_it==smap.end())
	{
      cerr << "Attempting to load the new carmen log format" << endl;
      laser_it=smap.find(std::string("ROBOTLASER1"));
      assert(laser_it!=smap.end());
    }
    const RangeSensor* rangeSensor=dynamic_cast<const RangeSensor*>((laser_it->second));
    assert(rangeSensor && rangeSensor->beams().size());
    
    m_beams=static_cast<unsigned int>(rangeSensor->beams().size());
    double* angles=new double[rangeSensor->beams().size()];
    for (unsigned int i=0; i<m_beams; i++)
	{
      angles[i]=rangeSensor->beams()[i].pose.theta;
    }
    m_matcher.setLaserParameters(m_beams, angles, rangeSensor->getPose());
    delete [] angles;
  }
  

  /*
  @desc GridFastSLAM初始化
  主要是用来初始化各个粒子的一些信息。
  */
  void GridSlamProcessor::init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta, OrientedPoint initialPose)
  {
  
	//设置地图大小和分辨率
	m_xmin=xmin;
    m_ymin=ymin;
    m_xmax=xmax;
    m_ymax=ymax;
    m_delta=delta;
    
	/*设置每个粒子的初始值*/
    m_particles.clear();
    TNode* node=new TNode(initialPose, 0, 0, 0);

    //粒子对应的地图进行初始化 用两个地图来进行初始化 一个高分辨率地图 一个低分辨率地图
    //高分辨率地图由自己指定 低分辨率地图固定为0.1m
    ScanMatcherMap lmap(Point(xmin+xmax, ymin+ymax)*.5, xmax-xmin, ymax-ymin, delta);
    ScanMatcherMap lowMap(Point(xmin+xmax,ymin+ymax)*0.5,xmax-xmin,ymax-ymin,0.1);
    for (unsigned int i=0; i<size; i++)
	{
      m_particles.push_back(Particle(lmap,lowMap));
      //m_particles.push_back(Particle(lmap));
      m_particles.back().pose=initialPose;
      m_particles.back().previousPose=initialPose;
      m_particles.back().setWeight(0);
      m_particles.back().previousIndex=0;
      
      // we use the root directly
      m_particles.back().node= node;
    }

    m_neff=(double)size;
    m_count=0;
    m_readingCount=0;
    m_linearDistance=m_angularDistance=0;
  }

  /*
  @desc 处理真实位置  主要用于仿真。当里程计的误差为0时使用。
  */
  void GridSlamProcessor::processTruePos(const OdometryReading& o)
  {
    const OdometrySensor* os=dynamic_cast<const OdometrySensor*>(o.getSensor());
    if (os && os->isIdeal() && m_outputStream){
      m_outputStream << setiosflags(ios::fixed) << setprecision(3);
      m_outputStream <<  "SIMULATOR_POS " <<  o.getPose().x << " " << o.getPose().y << " " ;
      m_outputStream << setiosflags(ios::fixed) << setprecision(6) << o.getPose().theta << " " <<  o.getTime() << endl;
    }
  }
  
  
  /*
   * 在scanmatcherprocessor里面也有一个这样的函数 但是那个函数是没有使用的
  @desc 处理一帧激光数据 这里是gmapping算法的主要函数。
  在这个函数里面调用其他的函数，包括里程计预测、激光测量更新、粒子采样等等步骤。
  
  主要步骤如下：
  利用运动模型更新里程计分布
  利用最近的一次观测来提高proposal分布。(scan-match)
  利用proposal分布+激光雷达数据来确定各个粒子的权重
  对粒子进行重采样
  
  */
  bool GridSlamProcessor::processScan(const RangeReading & reading, int adaptParticles)
  {
    /**retireve the position from the reading, and compute the odometry*/
    /*得到当前的里程计的位置*/
	OrientedPoint relPose=reading.getPose();
    //relPose.y = m_odoPose.y;
    
	/*m_count表示这个函数被调用的次数 如果是第0次调用,则所有的位姿都是一样的*/
	if (!m_count)
	{
      m_lastPartPose=m_odoPose=relPose;
    }
    
    //write the state of the reading and update all the particles using the motion model
	/*对于每一个粒子，都从里程计运动模型中采样，得到车子的初步估计位置  这一步对应于   里程计的更新 */
    int tmp_size = m_particles.size();

//这个for循环显然可以用OpenMP进行并行化
//#pragma omp parallel for
    for(int i = 0; i < tmp_size;i++)
    {
        OrientedPoint& pose(m_particles[i].pose);
        pose = m_motionModel.drawFromMotion(m_particles[i],relPose,m_odoPose);
    }

    //invoke the callback
    /*回调函数  实际上什么都没做*/
    onOdometryUpdate();
    
    // accumulate the robot translation and rotation
    /*根据两次里程计的数据 计算出来机器人的线性位移和角度位移的累积值 m_odoPose表示上一次的里程计位姿  relPose表示新的里程计的位姿*/
    OrientedPoint move=relPose-m_odoPose;
    move.theta=atan2(sin(move.theta), cos(move.theta));

    //统计机器人在进行激光雷达更新之前 走了多远的距离 以及　平移了多少的角度
    m_linearDistance+=sqrt(move*move);
    m_angularDistance+=fabs(move.theta);

//    cerr <<"linear Distance:"<<m_linearDistance<<endl;
//    cerr <<"angular Distance:"<<m_angularDistance<<endl;
    
    // if the robot jumps throw a warning
    /*
     * 如果机器人在走了m_distanceThresholdCheck这么远的距离都没有进行激光雷达的更新
     * 则需要进行报警。这个误差很可能是里程计或者激光雷达的BUG造成的。
     * 例如里程计数据出错 或者 激光雷达很久没有数据等等
     * 每次进行激光雷达的更新之后 m_linearDistance这个参数就会清零
     */
    if (m_linearDistance>m_distanceThresholdCheck)
    {
      cerr << "***********************************************************************" << endl;
      cerr << "********** Error: m_distanceThresholdCheck overridden!!!! *************" << endl;
      cerr << "m_distanceThresholdCheck=" << m_distanceThresholdCheck << endl;
      cerr << "Old Odometry Pose= " << m_odoPose.x << " " << m_odoPose.y 
	   << " " <<m_odoPose.theta << endl;
      cerr << "New Odometry Pose (reported from observation)= " << relPose.x << " " << relPose.y 
	   << " " <<relPose.theta << endl;
      cerr << "***********************************************************************" << endl;
      cerr << "** The Odometry has a big jump here. This is probably a bug in the   **" << endl;
      cerr << "** odometry/laser input. We continue now, but the result is probably **" << endl;
      cerr << "** crap or can lead to a core dump since the map doesn't fit.... C&G **" << endl;
      cerr << "***********************************************************************" << endl;
    }
    
    //更新 把当前的位置赋值给旧的位置
    m_odoPose=relPose;
    
    bool processed=false;

    // process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
	/*只有当机器人走过一定的距离  或者 旋转过一定的角度  或者过一段指定的时间才处理激光数据*/
    if (! m_count 
	|| m_linearDistance>=m_linearThresholdDistance 
	|| m_angularDistance>=m_angularThresholdDistance
    || (period_ >= 0.0 && (reading.getTime() - last_update_time_) > period_))
	{
          last_update_time_ = reading.getTime();

          std::cout <<std::endl<<"Start to Process Scan##################"<<std::endl;

          if (m_outputStream.is_open())
          {
              m_outputStream << setiosflags(ios::fixed) << setprecision(6);
              m_outputStream << "FRAME " <<  m_readingCount;
              m_outputStream << " " << m_linearDistance;
              m_outputStream << " " << m_angularDistance << endl;
          }

//          if (m_infoStream)
//          {
//              m_infoStream << "update frame " <<  m_readingCount << endl
//                           << "update ld=" << m_linearDistance << " ad=" << m_angularDistance << endl;

//              m_infoStream << "FRAME " <<  m_readingCount<<endl;
//              m_infoStream <<"Scan-Match Number: "<<m_count<<endl;
//          }
//          cerr << "Laser Pose= " << reading.getPose().x << " " << reading.getPose().y
//                 << " " << reading.getPose().theta << endl;

          //this is for converting the reading in a scan-matcher feedable form
          /*复制一帧数据 把激光数据转换为scan-match需要的格式*/
//          if(reading.getSize() != m_beams)
//          {
//              cerr<<"********************************************"<<endl;
//              cerr<<"********************************************"<<endl;
//              cerr<<"reading_size:"<<reading.getSize()<<"  m_beams:"<<m_beams<<endl;
//              cerr<<"********************************************"<<endl;
//              cerr<<"********************************************"<<endl;
//          }
          int beam_number = reading.getSize();
          double * plainReading = new double[beam_number];
          for(unsigned int i=0; i<beam_number; i++)
          {
            plainReading[i]=reading.m_dists[i];
          }
          //这个备份主要是用来储存的。
          RangeReading* reading_copy;

          //champion_nav_msgs激光数据
          if(reading.m_angles.size() == reading.m_dists.size())
          {
              reading_copy = new RangeReading(beam_number,
                                   &(reading.m_dists[0]),
                                   &(reading.m_angles[0]),
                                   static_cast<const RangeSensor*>(reading.getSensor()),
                                   reading.getTime());

          }
          //ros的激光数据
          else
          {
              reading_copy = new RangeReading(beam_number,
                                   &(reading.m_dists[0]),
                                   static_cast<const RangeSensor*>(reading.getSensor()),
                                   reading.getTime());
          }
          /*如果不是第一帧数据*/
          if (m_count>0)
          {
            /*
            为每个粒子进行scanMatch，计算出来每个粒子的最优位姿，同时计算改最优位姿的得分和似然  对应于gmapping论文中的用最近的一次测量计算proposal的算法
            这里面除了进行scanMatch之外，还对粒子进行了权重的计算，并计算了粒子的有效区域 但不进行内存分配 内存分配在resample()函数中
            这个函数在gridslamprocessor.hxx里面。
            */
            scanMatch(plainReading);

            //至此 关于proposal的更新完毕了，接下来是计算权重
            onScanmatchUpdate();

            /*
            由于scanMatch中对粒子的权重进行了更新，那么这个时候各个粒子的轨迹上的累计权重都需要重新计算
            这个函数即更新各个粒子的轨迹上的累计权重是更新
            GridSlamProcessor::updateTreeWeights(bool weightsAlreadyNormalized) 函数在gridslamprocessor_tree.cpp里面实现
            */
            updateTreeWeights(false);

            /*
             * 粒子重采样  根据neff的大小来进行重采样  不但进行了重采样，也对地图进行更新
             * GridSlamProcessor::resample 函数在gridslamprocessor.hxx里面实现
             */
            std::cerr<<"plainReading:"<<m_beams<<std::endl;
            resample(plainReading, adaptParticles, reading_copy);
          }
          /*如果是第一帧激光数据*/
          else
          {
            //如果是第一帧数据，则可以直接计算activeArea。因为这个时候，对机器人的位置是非常确定的，就是(0,0,0)
            for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
            {
                m_matcher.invalidateActiveArea();
                m_matcher.computeActiveArea(it->map, it->pose, plainReading);
                m_matcher.registerScan(it->map, it->pose, plainReading);
                //m_matcher.registerScan(it->lowResolutionMap,it->pose,plainReading);

                //为每个粒子创建路径的第一个节点。该节点的权重为0,父节点为it->node(这个时候为NULL)。
                //因为第一个节点就是轨迹的根，所以没有父节点
                TNode* node=new	TNode(it->pose, 0., it->node,  0);
                node->reading = reading_copy;
                it->node=node;
            }
         }
          //		cerr  << "Tree: normalizing, resetting and propagating weights at the end..." ;
         //进行重采样之后，粒子的权重又会发生变化，因此需要再次更新粒子轨迹的累计权重
         //GridSlamProcessor::updateTreeWeights(bool weightsAlreadyNormalized) 函数在gridslamprocessor_tree.cpp里面实现
         updateTreeWeights(false);
          //		cerr  << ".done!" <<endl;

        delete [] plainReading;
        m_lastPartPose=m_odoPose; //update the past pose for the next iteration

        //机器人累计行走的多远的路程没有进行里程计的更新 每次更新完毕之后都要把这个数值清零
        m_linearDistance=0;
        m_angularDistance=0;

        m_count++;
        processed=true;

          //keep ready for the next step
        for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
        {
            it->previousPose=it->pose;
        }
    }
    m_readingCount++;
    return processed;
 }
  
  
  std::ofstream& GridSlamProcessor::outputStream(){
    return m_outputStream;
  }
  
  std::ostream& GridSlamProcessor::infoStream(){
    return m_infoStream;
  }
  
  
  
  /*
  得到累计权重最大的粒子的下标
  注意不是当前的最大权重的粒子，是累计权重最大的粒子
  累计权重即该粒子在各个时刻的权重之和(轨迹上的各个节点的权重之和)
  */
  int GridSlamProcessor::getBestParticleIndex() const
  {
    unsigned int bi=0;
    double bw=-std::numeric_limits<double>::max();
    for (unsigned int i=0; i<m_particles.size(); i++)
    {
      if (bw<m_particles[i].weightSum)
	  {
		bw=m_particles[i].weightSum;
		bi=i;
      }
    }
    return (int) bi;
  }

  void GridSlamProcessor::onScanmatchUpdate(){}
  void GridSlamProcessor::onResampleUpdate(){}
  void GridSlamProcessor::onOdometryUpdate(){}

  
};// end namespace




