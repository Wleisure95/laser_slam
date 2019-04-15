#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include <gmapping/utils/point.h>
#include <gmapping/utils/stat.h>
#include <gmapping/utils/macro_params.h>

namespace  GMapping { 

/*里程计运动模型*/
struct MotionModel{
	
	/*给点当前坐标 里程计信息 求去新的位置*/
	OrientedPoint drawFromMotion(const OrientedPoint& p, double linearMove, double angularMove) const;
	
	/*给点当前坐标 这一次的里程计信息  上一次的里程计信息  计算出来新的位置*/
	OrientedPoint drawFromMotion(const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold) const;
	
	Covariance3 gaussianApproximation(const OrientedPoint& pnew, const OrientedPoint& pold) const;
	double srr, str, srt, stt;
	
	/*
	@srr  线性运动造成的线性误差的方差
	@srt  线性运动造成的角度误差的方差
	@str  旋转运动造成的线性误差的方差
	@stt  旋转运动造成的角度误差的方差
	*/
};

};

#endif
