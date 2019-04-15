#ifndef GRIDSLAMPROCESSOR_H
#define GRIDSLAMPROCESSOR_H

#include <climits>
#include <limits>
#include <fstream>
#include <vector>
#include <deque>
#include <omp.h>
#include "../include/gmapping/particlefilter/particlefilter.h"
#include "../include/gmapping/utils/point.h"
#include "../include/gmapping/utils/macro_params.h"
#include "../include/gmapping/log/sensorlog.h"
#include "../include/gmapping/sensor/sensor_range/rangesensor.h"
#include "../include/gmapping/sensor/sensor_range/rangereading.h"
#include "../include/gmapping/scanmatcher/scanmatcher.h"
#include "motionmodel.h"


namespace GMapping {

  /**This class defines the basic GridFastSLAM algorithm.  It
     implements a rao blackwellized particle filter. Each particle
     has its own map and robot pose.<br> This implementation works
     as follows: each time a new pair odometry/laser reading is
     received, the particle's robot pose is updated according to the
     motion model.  This pose is subsequently used for initalizing a
     scan matching algorithm.  The scanmatcher performs a local
     optimization for each particle.  It is initialized with the
     pose drawn from the motion model, and the pose is corrected
     according to the each particle map.<br>
     In order to avoid unnecessary computation the filter state is updated 
     only when the robot moves more than a given threshold.
	 
	 这个类实现了一个GridFastSLAM算法。实现了一个RBPF，每个粒子都拥有自己的地图和机器人位姿。
	 工作流程如下：
	 每当收到里程计数据和激光雷达的数据之后，每个粒子的位姿根据运动模型来更新。
	 根据运动模型更新得到的新的位置随后被用来初始化scan-match算法。
	 scan-matcher为每个粒子执行了一个局部优化算法。
	 scan-matcher被用运动模型得到的位置来初始化，然后根据自己的地图来优化位置。
	 
	 为了减少不必要的计算量，滤波器的状态必须要在机器人运动了一段距离之后才更新
	 
  */
  class GridSlamProcessor
  {
  public:

    
    /**This class defines the the node of reversed tree in which the trajectories are stored.
       Each node of a tree has a pointer to its parent and a counter indicating the number of childs of a node.
       The tree is updated in a way consistent with the operation performed on the particles.
	   
	   //树的节点，一个树储存了一整条轨迹，一个节点表示这条轨迹中的其中一个点。
	   //因为FastSLAM是一个full SLAM的方法，因此它需要存储机器人的整条轨迹。
	   //轨迹上的一个节点(相当于一帧)存储了一下几个信息：
	     机器人的位姿
         该节点粒子的权重
         轨迹前面所有的节点的粒子的权重之和
		 指向父节点的指针
		 子节点的数量
	   
	   
   */
    struct TNode
    {
      /**Constructs a node of the trajectory tree.
       @param pose:      the pose of the robot in the trajectory
       @param weight:    the weight of the particle at that point in the trajectory
       @param accWeight: the cumulative weight of the particle
       @param parent:    the parent node in the tree
       @param childs:    the number of childs
      */
      TNode(const OrientedPoint& pose, double weight, TNode* parent=0, unsigned int childs=0);

      /**Destroys a tree node, and consistently updates the tree. If a node whose parent has only one child is deleted,
       also the parent node is deleted. This because the parent will not be reacheable anymore in the trajectory tree.*/
      ~TNode();

      /**The pose of the robot*/
	  //该节点机器人的位姿
      OrientedPoint pose; 
      
      /**The weight of the particle*/
	  //该节点粒子的权重
      double weight;

      /**The sum of all the particle weights in the previous part of the trajectory*/
      // 轨迹前面所有节点的粒子的权重之和 该节点的子节点纸盒
      double accWeight;

      double gweight;

      /**The parent*/
      //指向父节点的指针 这里没有指向子节点的指针 因此每个粒子的路径都是记录最近的一个点，然后通过parent指针遍历整条路径
      TNode* parent;

      /**The range reading to which this node is associated*/
      // 该节点激光雷达的读数
      const RangeReading* reading;

      /**The number of childs*/
      // 该节点的子节点的数量
      unsigned int childs;

      /**counter in visiting the node (internally used)*/
      mutable unsigned int visitCounter;

      /**visit flag (internally used)*/
      mutable bool flag;
    };
    
    typedef std::vector<GridSlamProcessor::TNode*> TNodeVector;
    typedef std::deque<GridSlamProcessor::TNode*> TNodeDeque;
    
    /**This class defines a particle of the filter. Each particle has a map, a pose, a weight and retains the current node in the trajectory tree*/
    /*
     * 粒子滤波器中的粒子结构体  每个粒子有自己的地图、位姿、权重、轨迹
     * 轨迹是按照时间顺序排列的，叶子节点表示最近的节点
     */
    struct Particle
    {
      /**constructs a particle, given a map
	 @param map: the particle map
      */
      Particle(const ScanMatcherMap& map);

      //给定两个地图 一个高分辨率地图 一个低分辨率地图
      Particle(const ScanMatcherMap& map,const ScanMatcherMap& lowMap);

      /** @returns the weight of a particle */
      inline operator double() const {return weight;}
      /** @returns the pose of a particle */
      inline operator OrientedPoint() const {return pose;}
      /** sets the weight of a particle
	  @param w the weight
      */
      inline void setWeight(double w) {weight=w;}
      /** The map  地图 高分辨率地图*/
      ScanMatcherMap map;

      /*存储最近的N帧激光雷达的数据 用来生成临时地图 从而进行CSM*/
      std::vector<GMapping::RangeReading*> running_scans;

      /*低分辨率地图 用来进行CSM*/
      ScanMatcherMap lowResolutionMap;


	  
      /** The pose of the robot 机器人位姿*/
      OrientedPoint pose;

      /** The pose of the robot at the previous time frame (used for computing thr odometry displacements) */
      /*机器人上一帧的位姿 这个位姿是用来计算里程计的位移的*/
	  OrientedPoint previousPose;

      /** The weight of the particle */
      /*粒子的权重*/
	  double weight;

      /** The cumulative weight of the particle */
      /*粒子的累计权重*/
	  double weightSum;

      double gweight;

      /** The index of the previous particle in the trajectory tree */
      /*上一个粒子的下标*/
	  int previousIndex;

      /** Entry to the trajectory tree */
      // 该粒子对应的整条轨迹 记录的是粒子的最近的一个节点
      TNode* node; 
    };
	
    
    typedef std::vector<Particle> ParticleVector;
    
    /** Constructs a GridSlamProcessor, initialized with the default parameters */
    GridSlamProcessor();

    /** Constructs a GridSlamProcessor, whose output is routed to a stream.
     @param infoStr: the output stream
    */
    GridSlamProcessor(std::ostream& infoStr);
    
    /** @returns  a deep copy of the grid slam processor with all the internal structures.
    */
    GridSlamProcessor* clone() const;
    
    /**Deleted the gridslamprocessor*/
    virtual ~GridSlamProcessor();
    
    //methods for accessing the parameters
    void setSensorMap(const SensorMap& smap);
    void init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta, 
	      OrientedPoint initialPose=OrientedPoint(0,0,0));
    void setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt, 
			       int iterations, double likelihoodSigma=1, double likelihoodGain=1, unsigned int likelihoodSkip=0);
    void setMotionModelParameters(double srr, double srt, double str, double stt);
    void setUpdateDistances(double linear, double angular, double resampleThreshold);
    void setUpdatePeriod(double p) {period_=p;}

    
    //the "core" algorithm
	/*核心算法*/
	
	/*在里程计为理想的情况下(即仿真的情况下使用)*/
    void processTruePos(const OdometryReading& odometry);
    
	/*处理激光数据*/
	bool processScan(const RangeReading & reading, int adaptParticles=0);

    bool processScan(const double * ranges,const double *angles,int beams_cnt);
    

    /**This method copies the state of the filter in a tree.
     The tree is represented through reversed pointers (each node has a pointer to its parent).
     The leafs are stored in a vector, whose size is the same as the number of particles.
     @returns the leafs of the tree
    */
	//得到各个粒子的轨迹树的叶子节点
    TNodeVector getTrajectories() const;
	
    //这个函数是没有被调用的
    void integrateScanSequence(TNode* node);
    
    /**the scanmatcher algorithm*/
    ScanMatcher m_matcher;

    /**the stream used for writing the output of the algorithm*/
    std::ofstream& outputStream();
    /**the stream used for writing the info/debug messages*/
    std::ostream& infoStream();

    /**@returns the particles*/
    inline const ParticleVector& getParticles() const {return m_particles; }
    
    inline const std::vector<unsigned int>& getIndexes() const{return m_indexes; }

    int getBestParticleIndex() const;

    //callbacks
    virtual void onOdometryUpdate();
    virtual void onResampleUpdate();
    virtual void onScanmatchUpdate();
	
    //accessor methods
    /**the maxrange of the laser to consider */
    MEMBER_PARAM_SET_GET(m_matcher, double, laserMaxRange, protected, public, public);

    /**the maximum usable range of the laser. A beam is cropped to this value. [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, usableRange, protected, public, public);

    /**The sigma used by the greedy endpoint matching. [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher,double, gaussianSigma, protected, public, public);

    /**The sigma  of a beam used for likelihood computation [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher,double, likelihoodSigma, protected, public, public);

    /**The kernel in which to look for a correspondence[scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, int,    kernelSize, protected, public, public);

    /**The optimization step in rotation [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, optAngularDelta, protected, public, public);

    /**The optimization step in translation [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, optLinearDelta, protected, public, public);

    /**The number of iterations of the scanmatcher [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, unsigned int, optRecursiveIterations, protected, public, public);

    /**the beams to skip for computing the likelihood (consider a beam every likelihoodSkip) [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, unsigned int, likelihoodSkip, protected, public, public);

    /**translational sampling range for the likelihood [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, llsamplerange, protected, public, public);

    /**angular sampling range for the likelihood [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, lasamplerange, protected, public, public);

    /**translational sampling range for the likelihood [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, llsamplestep, protected, public, public);

    /**angular sampling step for the likelihood [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, double, lasamplestep, protected, public, public);

    /**generate an accupancy grid map [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, bool, generateMap, protected, public, public);

    /**enlarge the map when the robot goes out of the boundaries [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, bool, enlargeStep, protected, public, public);

    /**pose of the laser wrt the robot [scanmatcher]*/
    MEMBER_PARAM_SET_GET(m_matcher, OrientedPoint, laserPose, protected, public, public);


    /**odometry error in translation as a function of translation (rho/rho) [motionmodel]*/
    STRUCT_PARAM_SET_GET(m_motionModel, double, srr, protected, public, public);

    /**odometry error in translation as a function of rotation (rho/theta) [motionmodel]*/
    STRUCT_PARAM_SET_GET(m_motionModel, double, srt, protected, public, public);

    /**odometry error in rotation as a function of translation (theta/rho) [motionmodel]*/
    STRUCT_PARAM_SET_GET(m_motionModel, double, str, protected, public, public);

    /**odometry error in  rotation as a function of rotation (theta/theta) [motionmodel]*/
    STRUCT_PARAM_SET_GET(m_motionModel, double, stt, protected, public, public);
		
    /**minimum score for considering the outcome of the scanmatching good*/
    PARAM_SET_GET(double, minimumScore, protected, public, public);

  protected:
    /**Copy constructor*/
    GridSlamProcessor(const GridSlamProcessor& gsp);
 
    /**the laser beams*/
    unsigned int m_beams;
    double last_update_time_;
    double period_;

    //ADD 07-04-2017
    double LineAngle;
    //End ADD 07-04-2017
    
    /**the particles*/
    ParticleVector m_particles;

    /**the particle indexes after resampling (internally used)*/
    //重采样之后，剩余的粒子的下标。这个下标的是会重复的
    std::vector<unsigned int> m_indexes;

    /**the particle weights (internally used)*/
    //所有粒子的当前的权重
    std::vector<double> m_weights;
    
    /**the motion model*/
    MotionModel m_motionModel;

    /**this sets the neff based resampling threshold*/
    PARAM_SET_GET(double, resampleThreshold, protected, public, public);
      
    //state
    int  m_count, m_readingCount;
    OrientedPoint m_lastPartPose;
    OrientedPoint m_odoPose;
    OrientedPoint m_pose;
    double m_linearDistance, m_angularDistance;
    PARAM_GET(double, neff, protected, public);
      
    //processing parameters (size of the map)
    PARAM_GET(double, xmin, protected, public);
    PARAM_GET(double, ymin, protected, public);
    PARAM_GET(double, xmax, protected, public);
    PARAM_GET(double, ymax, protected, public);
    //processing parameters (resolution of the map)
    PARAM_GET(double, delta, protected, public);
	
    //registration score (if a scan score is above this threshold it is registered in the map)
    PARAM_SET_GET(double, regScore, protected, public, public);
    //registration score (if a scan score is below this threshold a scan matching failure is reported)
    PARAM_SET_GET(double, critScore, protected, public, public);
    //registration score maximum move allowed between consecutive scans
    PARAM_SET_GET(double, maxMove, protected, public, public);
	
    //process a scan each time the robot translates of linearThresholdDistance
    PARAM_SET_GET(double, linearThresholdDistance, protected, public, public);

    //process a scan each time the robot rotates more than angularThresholdDistance
    PARAM_SET_GET(double, angularThresholdDistance, protected, public, public);
    
    //smoothing factor for the likelihood
    PARAM_SET_GET(double, obsSigmaGain, protected, public, public);
	
    //stream in which to write the gfs file
    std::ofstream m_outputStream;

    // stream in which to write the messages
    std::ostream& m_infoStream;
    
    
    // the functions below performs side effect on the internal structure,
    //should be called only inside the processScan method
  private:
    
    /**scanmatches all the particles*/
    inline void scanMatch(const double *plainReading);
    /**normalizes the particle weights*/
    inline void normalize();
    
    // return if a resampling occured or not
    inline bool resample(const double* plainReading, int adaptParticles, 
			 const RangeReading* rr=0);
    
    //tree utilities
    
    void updateTreeWeights(bool weightsAlreadyNormalized = false);
    void resetTree();
    double propagateWeights();
    
  };

typedef std::multimap<const GridSlamProcessor::TNode*, GridSlamProcessor::TNode*> TNodeMultimap;


  /*
   * 这个文件中实现的是gridslamprocessor.h里面的三个内联函数
  */


  /**Just scan match every single particle.
  If the scan matching fails, the particle gets a default likelihood.
  @desc 为每一个粒子都计算扫描匹配。扫描匹配即为在里程计的基础上，通过优化求得位姿
  这个函数是最终被使用的函数

  这个函数不但对每个位姿进行scan-match来优化，再优化之后，还会计算每个粒子的权重
  这里的权重用似然表示。
  这个函数实现的scan-match方法是cyrill stachniss在07年提出的论文里面的方法。
  理论上来说这个方法比GMapping的原来的那个方法好一些 但实际上好像差不多。
  */
//  inline void GridSlamProcessor::scanMatch(const double* plainReading)
//  {
//    // sample a new pose from each scan in the reference
//    /*每个粒子都要进行scan-match*/
//    double sumScore=0;
//    int particle_number = m_particles.size();

//    //用openMP的方式来进行并行化，因此这里不能用迭代器 只能用下标的方式进行访问
//    //并行话之后会把里面的循环均匀的分到各个不同的核里面去。
//  #pragma omp parallel for
//    for (int i = 0; i < particle_number;i++)
//    {
//      OrientedPoint corrected;
//      ScanMatcher::CovarianceMatrix cov;
//      double score, l;

//      //进行完scan-match之后 不是直接用最优的位姿 而是假设高斯分布然后计算加权和 这个是论文里面提出来的方法
//      score = m_matcher.optimize(corrected,cov,m_particles[i].map,m_particles[i].pose,plainReading);

//      /*矫正成功则更新位姿*/
//      if (score>m_minimumScore)
//      {
//        m_particles[i].pose = corrected;
//      }
//      /*扫描匹配不上 则使用里程计的数据 使用里程计数据不进行更新  因为在进行扫描匹配之前 里程计已经更新过了*/
//      else
//      {
//          //输出信息 这个在并行模式下可以会出现错位
//          if (m_infoStream)
//          {
//              m_infoStream << "Scan Matching Failed, using odometry. Likelihood=" << l <<std::endl;
//              m_infoStream << "lp:" << m_lastPartPose.x << " "  << m_lastPartPose.y << " "<< m_lastPartPose.theta <<std::endl;
//              m_infoStream << "op:" << m_odoPose.x << " " << m_odoPose.y << " "<< m_odoPose.theta <<std::endl;
//          }
//      }

//      //粒子的最优位姿计算了之后，重新计算粒子的权重(相当于粒子滤波器中的观测步骤，计算p(z|x,m))，粒子的权重由粒子的似然来表示。
//      /*
//       * 计算粒子的得分和权重(似然)   注意粒子的权重经过ScanMatch之后已经更新了
//       * 在论文中 例子的权重不是用最有位姿的似然值来表示的。
//       * 是用所有的似然值的和来表示的。
//       */
//      //这个计算权重的方法来时Cyrill Stachniss在论文中提到的方法。
//      OrientedPoint mean;
//      double w = m_matcher.likelihood(l,mean,cov,m_particles[i].map,m_particles[i].pose,plainReading);
//      l = w;

//      sumScore+=score;
//      m_particles[i].weight+=l;
//      m_particles[i].weightSum+=l;

//      //set up the selective copy of the active area
//      //by detaching the areas that will be updated
//      /*计算出来最优的位姿之后，进行地图的扩充  这里不会进行内存分配
//       *不进行内存分配的原因是这些粒子进行重采样之后有可能会消失掉，因此在后面进行冲采样的时候统一进行内存分配。
//       *理论上来说，这里的操作是没有必要的，因为后面的重采样的时候还会进行一遍
//       */
//      //m_matcher.invalidateActiveArea();
//      //m_matcher.computeActiveArea(m_particles[i].map, m_particles[i].pose, plainReading);
//    }
//    if (m_infoStream)
//      m_infoStream << "Average Scan Matching Score=" << sumScore/m_particles.size() << std::endl;
//  }


//上面定义的三个内联函数在这个头文件里面实现
//scanMatch normalize resample
#include "gridslamprocessor.hxx"

};

#endif
