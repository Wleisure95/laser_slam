
#ifdef MACOSX
// This is to overcome a possible bug in Apple's GCC.
#define isnan(x) (x==FP_NAN)
#endif

#include <omp.h>


/*
 * 这个文件中实现的是gridslamprocessor.h里面的三个内联函数
*/


/**Just scan match every single particle.
If the scan matching fails, the particle gets a default likelihood.
@desc 为每一个粒子都计算扫描匹配。扫描匹配即为在里程计的基础上，通过优化求得位姿
这个函数是最终被使用的函数

这个函数不但对每个位姿进行scan-match来优化，再优化之后，还会计算每个粒子的权重
这里的权重用似然表示。
*/
inline void GridSlamProcessor::scanMatch(const double* plainReading)
{
  // sample a new pose from each scan in the reference
  /*每个粒子都要进行scan-match*/
  double sumScore=0;
  int particle_number = m_particles.size();

  //用openMP的方式来进行并行化，因此这里不能用迭代器 只能用下标的方式进行访问
  //并行话之后会把里面的循环均匀的分到各个不同的核里面去。

//#pragma omp parallel for
  for (int i = 0; i < particle_number;i++)
  {
    OrientedPoint corrected;
    double score, l, s;

    /*进行scan-match 计算粒子的最优位姿 调用scanmatcher.cpp里面的函数 --这是gmapping本来的做法*/
    score=m_matcher.optimize(corrected, m_particles[i].map, m_particles[i].pose, plainReading);

    /*矫正成功则更新位姿*/
    if (score>m_minimumScore)
    {
      m_particles[i].pose = corrected;
    }
    /*扫描匹配不上 则使用里程计的数据 使用里程计数据不进行更新  因为在进行扫描匹配之前 里程计已经更新过了*/
    else
    {
        //输出信息 这个在并行模式下可以会出现错位
        if (m_infoStream)
        {
            m_infoStream << "Scan Matching Failed, using odometry. Likelihood=" << l <<std::endl;
            m_infoStream << "lp:" << m_lastPartPose.x << " "  << m_lastPartPose.y << " "<< m_lastPartPose.theta <<std::endl;
            m_infoStream << "op:" << m_odoPose.x << " " << m_odoPose.y << " "<< m_odoPose.theta <<std::endl;
        }
    }

    //粒子的最优位姿计算了之后，重新计算粒子的权重(相当于粒子滤波器中的观测步骤，计算p(z|x,m))，粒子的权重由粒子的似然来表示。
    /*
     * 计算粒子的得分和权重(似然)   注意粒子的权重经过ScanMatch之后已经更新了
     * 在论文中 例子的权重不是用最有位姿的似然值来表示的。
     * 是用所有的似然值的和来表示的。
     */
    m_matcher.likelihoodAndScore(s, l, m_particles[i].map, m_particles[i].pose, plainReading);

    sumScore+=score;
    m_particles[i].weight+=l;
    m_particles[i].weightSum+=l;

    //set up the selective copy of the active area
    //by detaching the areas that will be updated
    /*计算出来最优的位姿之后，进行地图的扩充  这里不会进行内存分配
     *不进行内存分配的原因是这些粒子进行重采样之后有可能会消失掉，因此在后面进行冲采样的时候统一进行内存分配。
     *理论上来说，这里的操作是没有必要的，因为后面的重采样的时候还会进行一遍
     */
    m_matcher.invalidateActiveArea();
    m_matcher.computeActiveArea(m_particles[i].map, m_particles[i].pose, plainReading);
  }
  if (m_infoStream)
    m_infoStream << "Average Scan Matching Score=" << sumScore/m_particles.size() << std::endl;
}


/*
@desc 把粒子的权重归一化
主要功能为归一化粒子的权重，同时计算出neff
*/
inline void GridSlamProcessor::normalize()
{
  //normalize the log m_weights
  double gain=1./(m_obsSigmaGain*m_particles.size());
  
  /*求所有粒子中的最大的权重*/
  double lmax= -std::numeric_limits<double>::max();
  for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
  {
    lmax=it->weight>lmax?it->weight:lmax;
  }
  //cout << "!!!!!!!!!!! maxwaight= "<< lmax << endl;
  
  /*权重以最大权重为中心的高斯分布*/
  m_weights.clear();
  double wcum=0;
  m_neff=0;
  for (std::vector<Particle>::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
  {
    m_weights.push_back(exp(gain*(it->weight-lmax)));
    wcum+=m_weights.back();
    //cout << "l=" << it->weight<< endl;
  }
  
  /*
  计算有效粒子数 和 归一化权重
  权重=wi/w
  neff = 1/w*w
  */
  m_neff=0;
  for (std::vector<double>::iterator it=m_weights.begin(); it!=m_weights.end(); it++)
  {
    *it=*it/wcum;
    double w=*it;
    m_neff+=w*w;
  }
  m_neff=1./m_neff;
  
}

/*
@desc 粒子滤波器重采样。

分为两步：
1.需要重采样，则所有保留下来的粒子的轨迹都加上一个新的节点，然后进行地图更新。
2.不需要冲采样，则所有的粒子的轨迹都加上一个新的节点，然后进行地图的更新

在重采样完毕之后，会调用registerScan函数来更新地图
*/
inline bool GridSlamProcessor::resample(const double* plainReading, int adaptSize, const RangeReading* reading)
{
  
  bool hasResampled = false;
  
  /*备份老的粒子的轨迹  即保留叶子节点 在增加新节点的时候使用*/
  TNodeVector oldGeneration;
  for (unsigned int i=0; i<m_particles.size(); i++)
  {
    oldGeneration.push_back(m_particles[i].node);
  }
  
  /*如果需要进行重采样*/
  if (m_neff<m_resampleThreshold*m_particles.size())
  {		
    
    if (m_infoStream)
      m_infoStream  << "*************RESAMPLE***************" << std::endl;
    
    //采取重采样方法决定，哪些粒子会保留  保留的粒子会返回下标.里面的下标可能会重复，因为有些粒子会重复采样
    //而另外的一些粒子会消失掉
    uniform_resampler<double, double> resampler;
    m_indexes=resampler.resampleIndexes(m_weights, adaptSize);
    
    if (m_outputStream.is_open())
	{
      m_outputStream << "RESAMPLE "<< m_indexes.size() << " ";
      for (std::vector<unsigned int>::const_iterator it=m_indexes.begin(); it!=m_indexes.end(); it++)
	  {
		m_outputStream << *it <<  " ";
      }
      m_outputStream << std::endl;
    }
    
    onResampleUpdate();
    //BEGIN: BUILDING TREE


    //重采样之后的粒子
    ParticleVector temp;
    unsigned int j=0;
	
	//要删除的粒子下标
    std::vector<unsigned int> deletedParticles;  		//this is for deleteing the particles which have been resampled away.
	
    //枚举每一个要被保留的粒子
    for (unsigned int i=0; i<m_indexes.size(); i++)
	{
      //统计要被删除的粒子
      while(j<m_indexes[i])
	  {
		deletedParticles.push_back(j);
		j++;
	  }
      if (j==m_indexes[i])
	  j++;
  
      //得到当前的保留的粒子
      Particle & p=m_particles[m_indexes[i]];
	  
      //每一个需要保留下来的粒子都需要在路径中增加一个新的节点
      TNode* node=0;
      TNode* oldNode=oldGeneration[m_indexes[i]];
	  
	  //创建一个新的节点 改节点的父节点为oldNode
      node=new	TNode(p.pose, 0, oldNode, 0);
      node->reading=reading;
      
	  //这个要保留下来的粒子，要保留的粒子的下标为m_indexs
      temp.push_back(p);
      temp.back().node=node;
      temp.back().previousIndex=m_indexes[i];
    }

    while(j<m_indexes.size())
	{
      deletedParticles.push_back(j);
      j++;
    }
	
	//把要删除的粒子的Node都删除掉，Node表示轨迹的起点(最新的点)
    std::cerr <<  "Deleting Nodes:";
    for (unsigned int i=0; i<deletedParticles.size(); i++)
	{
      std::cerr <<" " << deletedParticles[i];
      delete m_particles[deletedParticles[i]].node;
      m_particles[deletedParticles[i]].node=0;
    }
	
    std::cerr  << " Done" <<std::endl;
    std::cerr << "Deleting old particles..." ;
    std::cerr << "Done" << std::endl;
	
    //清楚全部的粒子 然后从tmp中读取保留下来的粒子
    m_particles.clear();
	
    //枚举要保留下来的所有的粒子 每个粒子都需要更新地图
    std::cerr << "Copying Particles and  Registering  scans...";

    //对于保留下来的粒子进行更新 这里是可以并行化操作的。
    //在并行化操作里面 m_particles.push_back()会报错 因此并行化 需要把push_back()提出来。
    //在外面的的for循环进行
    int tmp_size = temp.size();
//#pragma omp parallel for
    for(int i = 0; i<tmp_size;i++)
    {
        //对保留下来的粒子数据进行更新
        //每个粒子的权重都设置为相同的值
        temp[i].setWeight(0);

        //为每个粒子更新running_scans

        //增加了一帧激光数据 因此需要更新地图
        m_matcher.registerScan(temp[i].map,temp[i].pose,plainReading);
        //m_matcher.registerScan(temp[i].lowResolutionMap,temp[i].pose,plainReading);
    }

    //提取出来 防止并行优化时报错
    for(int i = 0; i< tmp_size;i++)
        m_particles.push_back(temp[i]);

    std::cerr  << " Done" <<std::endl;
    hasResampled = true;
  } 
  /*否则的话，进行扫描匹配*/
  else 
  {
	//不进行重采样的话，权值不变。只为轨迹创建一个新的节点

    //为每个粒子更新地图 同样可以并行化
    int particle_size = m_particles.size();
//#pragma omp parallel for
    for(int i = 0; i < particle_size;i++)
    {
        //创建一个新的树节点
        TNode* node = 0;
        node = new TNode(m_particles[i].pose,0.0,oldGeneration[i],0);

        //把这个节点接入到树中
        node->reading = reading;
        m_particles[i].node = node;

        //更新各个例子的地图
        m_matcher.invalidateActiveArea();
        m_matcher.registerScan(m_particles[i].map, m_particles[i].pose, plainReading);
        m_particles[i].previousIndex = i;
    }
    std::cerr<<std::endl;

//    int index=0;
//    TNodeVector::iterator node_it=oldGeneration.begin();
//    for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
//	{
//      //create a new node in the particle tree and add it to the old tree
//      //新建一个树的节点。
//      //BEGIN: BUILDING TREE
//      TNode* node=0;
//      node=new TNode(it->pose, 0.0, *node_it, 0);
      
//      //node->reading=0;
//      node->reading=reading;
//      it->node=node;

//      //END: BUILDING TREE
//      //粒子的轨迹更新了之后，对应的地图也需要更新
//      m_matcher.invalidateActiveArea();
//      m_matcher.registerScan(it->map, it->pose, plainReading);
//      it->previousIndex=index;
//      index++;
//      node_it++;
//    }
    std::cerr  << "Done" <<std::endl;
    
  }
  //END: BUILDING TREE
  
  return hasResampled;
}
