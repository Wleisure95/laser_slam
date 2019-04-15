#include <string>
#include <deque>
#include <list>
#include <map>
#include <set>
#include <fstream>
//#include <gsl/gsl_blas.h>

#include "../include/gmapping/utils/stat.h"
#include "../include/gmapping/gridfastslam/gridslamprocessor.h"

namespace GMapping {

using namespace std;

/*
 * 树的一个节点的构造函数 一个节点相当于一个粒子
 * p粒子的位姿
 * w粒子的权重
 * n粒子的父节点
 * c粒子的子节点的数量
*/
GridSlamProcessor::TNode::TNode(const OrientedPoint& p, double w, TNode* n, unsigned int c)
{
	pose=p;
	weight=w;
	childs=c;
	parent=n;
	reading=0;
	gweight=0;

	if (n)
	{
		n->childs++;
	}
	flag=0;
	accWeight=0;
}

GridSlamProcessor::TNode::~TNode()
{
	if (parent && (--parent->childs)<=0)
		delete parent;
	assert(!childs);
}


//BEGIN State Save/Restore
/*
得到所有粒子的轨迹的起点(得到了轨迹的起点，就可以得到一整条轨迹)
*/
GridSlamProcessor::TNodeVector GridSlamProcessor::getTrajectories() const
{
  TNodeVector v;
  TNodeMultimap parentCache;
  TNodeDeque border;
	
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++)
    {
		TNode* node=it->node;
        while(node)
        {
			node->flag=false;
			node=node->parent;
		}
	}
	
    for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++)
    {
		TNode* newnode=new TNode(* (it->node) );
		
		v.push_back(newnode);
		assert(newnode->childs==0);
        if (newnode->parent)
        {
			parentCache.insert(make_pair(newnode->parent, newnode));
			//cerr << __PRETTY_FUNCTION__ << ": node " << newnode->parent << " flag=" << newnode->parent->flag<< endl;
            if (! newnode->parent->flag)
            {
				//cerr << __PRETTY_FUNCTION__ << ": node " << newnode->parent << " flag=" << newnode->parent->flag<< endl;
				newnode->parent->flag=true;
				border.push_back(newnode->parent);
			}
		}
	}
	
	//cerr << __PRETTY_FUNCTION__ << ": border.size(INITIAL)=" << border.size() << endl;
	//cerr << __PRETTY_FUNCTION__ << ": parentCache.size()=" << parentCache.size() << endl;
    while (! border.empty())
    {
		//cerr << __PRETTY_FUNCTION__ << ": border.size(PREPROCESS)=" << border.size() << endl;
		//cerr << __PRETTY_FUNCTION__ << ": parentCache.size(PREPROCESS)=" << parentCache.size() << endl;
		const TNode* node=border.front();
		//cerr << __PRETTY_FUNCTION__ << ": node " << node << endl;
		border.pop_front();
		if (! node)
			continue;
			
		TNode* newnode=new TNode(*node);
		node->flag=false;
		
		//update the parent of all of the referring childs 
		pair<TNodeMultimap::iterator, TNodeMultimap::iterator> p=parentCache.equal_range(node);
		double childs=0;
        for (TNodeMultimap::iterator it=p.first; it!=p.second; it++)
        {
			assert(it->second->parent==it->first);
			(it->second)->parent=newnode;
			//cerr << "PS(" << it->first << ", "<< it->second << ")";
			childs++;
		}
		////cerr << endl;
		parentCache.erase(p.first, p.second);
		//cerr << __PRETTY_FUNCTION__ << ": parentCache.size(POSTERASE)=" << parentCache.size() << endl;
		assert(childs==newnode->childs);
		
		//unmark the node
        if ( node->parent )
        {
			parentCache.insert(make_pair(node->parent, newnode));
            if(! node->parent->flag)
            {
				border.push_back(node->parent);
				node->parent->flag=true;
			}	
		}
		//insert the parent in the cache
	}
	//cerr << __PRETTY_FUNCTION__ << " : checking cloned trajectories" << endl;
    for (unsigned int i=0; i<v.size(); i++)
    {
		TNode* node= v[i];
        while (node)
        {
			//cerr <<".";
			node=node->parent;
		}
		//cerr << endl;
	}	
	
	return v;

}

/**
 * @brief GridSlamProcessor::integrateScanSequence
 * 集成所有的scan
 * 这个函数没有被调用过。
 * @param node
 */
void GridSlamProcessor::integrateScanSequence(GridSlamProcessor::TNode* node)
{
	//reverse the list  
	//整条取反 因为路径的存储是按照时间顺序排列的。
	//我们只能得到最近时刻的节点，然后反推出来整条路径
    //但是如果我们想要集成所有的scan的话，我们必须从时间最早的节点开始。所以需要把整条路径反过来。
	TNode* aux=node;
	TNode* reversed=0;
	double count=0;
	while(aux!=0)
	{
		TNode * newnode=new TNode(*aux);
		newnode->parent=reversed;
		reversed=newnode;
		aux=aux->parent;
		count++;
	}
	
	//attach the path to each particle and compute the map;
	if (m_infoStream )
		m_infoStream << "Restoring State Nodes=" <<count << endl;
		
		
	aux=reversed;
	bool first=true;
	double oldWeight=0;
	OrientedPoint oldPose;
	while (aux!=0)
	{
		if (first)
		{
			oldPose=aux->pose;
			first=false;
			oldWeight=aux->weight;
		}
		
		OrientedPoint dp=aux->pose-oldPose;
		double dw=aux->weight-oldWeight;
		oldPose=aux->pose;
		

		double * plainReading = new double[m_beams];
		for(unsigned int i=0; i<m_beams; i++)
            plainReading[i]=aux->reading->m_dists[i];
		
		for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
		{
			//compute the position relative to the path;
			double s=sin(oldPose.theta-it->pose.theta),
			       c=cos(oldPose.theta-it->pose.theta);
			
			it->pose.x+=c*dp.x-s*dp.y;
			it->pose.y+=s*dp.x+c*dp.y;
			it->pose.theta+=dp.theta;
			it->pose.theta=atan2(sin(it->pose.theta), cos(it->pose.theta));
			
			//register the scan
           // m_matcher.invalidateActiveArea();
			m_matcher.computeActiveArea(it->map, it->pose, plainReading);
			it->weight+=dw;
			it->weightSum+=dw;

			// this should not work, since it->weight is not the correct weight!
			//			it->node=new TNode(it->pose, it->weight, it->node);
			it->node=new TNode(it->pose, 0.0, it->node);
			//update the weight
		}
		
		delete [] plainReading;
		aux=aux->parent;
	}
	
	//destroy the path
	aux=reversed;
	while (reversed)
	{
		aux=reversed;
		reversed=reversed->parent;
		delete aux;
	}
}

//END State Save/Restore

//BEGIN

/*
更新权重
调用这个函数的目的是因此
*/
void  GridSlamProcessor::updateTreeWeights(bool weightsAlreadyNormalized)
{

  if (!weightsAlreadyNormalized) 
  {
    normalize();
  }
  resetTree();
  propagateWeights();
}

/*把所有粒子的所有的轨迹中各个节点的accWeight清零*/
void GridSlamProcessor::resetTree()
{
  // don't calls this function directly, use updateTreeWeights(..) !

	for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
	{
		TNode* n=it->node;
		while (n)
		{
			n->accWeight=0;
			n->visitCounter=0;
			n=n->parent;
		}
	}
}


/*
 * 这个函数被下面的函数调用
 * 递归函数，
 * weight表示节点n的所有子节点的累计权重
 * 迭代求一条路径上所有点的累计权重
 * 节点的累计权重表示该节点的所有子节点的权重的和
*/
double propagateWeight(GridSlamProcessor::TNode* n, double weight)
{
	if (!n)
		return weight;
	
	double w=0;
	n->visitCounter++;
	n->accWeight+=weight;
	
	if (n->visitCounter==n->childs)
	{
		w=propagateWeight(n->parent,n->accWeight);
	}
	assert(n->visitCounter<=n->childs);
	return w;
}

/*
这个函数被updateTreeWeights()调用
*/
double GridSlamProcessor::propagateWeights()
{
  // don't calls this function directly, use updateTreeWeights(..) !

        // all nodes must be resetted to zero and weights normalized

    // the accumulated weight of the root
    // 求所有根节点的累计权重之和
	double lastNodeWeight=0;

	// sum of the weights in the leafs
    // 所有叶子节点的权重 也就是m_weights数组里面所有元素的和
    double aw=0;

	std::vector<double>::iterator w=m_weights.begin();
	for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
	{

        //求解所有叶子节点的累计权重
		double weight=*w;
		aw+=weight;
		
        //叶子节点的子节点累计权重就等于自己的权重 因为它没有子节点
        //每一个粒子的路径都是从叶子节点开始的，得到了叶子节点，就得到了路径
		TNode * n=it->node;
		n->accWeight=weight;

		lastNodeWeight+=propagateWeight(n->parent,n->accWeight);
		
		w++;
	}
	
    if (fabs(aw-1.0) > 0.0001 || fabs(lastNodeWeight-1.0) > 0.0001)
    {
	  cerr << "ERROR: ";
	  cerr << "root->accWeight=" << lastNodeWeight << "    sum_leaf_weights=" << aw << endl;
	  assert(0);         
	}
	return lastNodeWeight;
}

};

//END
