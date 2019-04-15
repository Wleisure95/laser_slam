#include <cstdlib>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/time.h>

#include <list>
#include "../include/gmapping/scanmatcher/icp.h"

using namespace GMapping;
using namespace std;

/*icp算法的测试文件  基本上没什么用*/

typedef std::list<PointPair> PointPairList;
std::vector<Point> origin_p1,origin_p2;

PointPairList generateRandomPointPairs(int size, OrientedPoint t, double noise=0.)
{
	PointPairList ppl;
	double s=sin(t.theta), c=cos(t.theta);
    for (int i=0; i<size; i++)
    {
		Point noiseDraw(noise*(drand48()-.5),noise*(drand48()-.5));
		PointPair pp;
        //随机生成第一帧数据
		pp.first.x=100.*(drand48()-.5)+200;
		pp.first.y=10.*(drand48()-.5);

        Point p1(pp.first.x,pp.first.y);
        origin_p1.push_back(p1);

        //第二帧的数据等于第一帧的数据上加上t 然后加上噪声
        //也就是说这两帧数据的转换关系就是t。
        pp.second.x= c*pp.first.x-s*pp.first.y;
		pp.second.y= s*pp.first.x+c*pp.first.y;
		pp.second=pp.second+t+noiseDraw;
		//cerr << "p1=" << pp.first.x << " " << pp.first.y << endl;
		//cerr << "p2=" << pp.second.x << " " << pp.second.y << endl;


        Point p2(pp.second.x,pp.second.y);
        origin_p2.push_back(p2);


		ppl.push_back(pp);
	}
	return ppl;
}



int main(int argc, const char ** argv)
{

    int i;
    double angle = -180;
    cout <<"{";
    while(angle < 180)
    {
        cout <<sin(angle/180*3.1415926)<<",";
        if(i++ >= 100)
        {
            i = 0;
            cout <<endl;
        }
        angle += 0.1;
    }
    cout <<"}"<<endl;


//    while (1)
//    {
//        clock_t st,ed;
//        struct timeval t_start,t_end;
//		OrientedPoint t;
//		int size;
//		cerr << "Insert size, t.x, t.y, t.theta" << endl;
//		cin >> size >> t.x >> t.y >> t.theta;
//        PointPairList ppl=generateRandomPointPairs(size, t, 3.0);
//		OrientedPoint tc;
//		OrientedPoint ttot(0.,0.,0.);
//		bool method=true;
//        while(1)
//        {
//			char buf[10];
//			cerr << "iterate?" << endl;
//			cin.getline(buf,10);
//			if (buf[0]=='n')
//				method=false;
//			else if (buf[0]=='l')
//				method=true;
//			else if (buf[0]!=char(0))
//				break;

//            //输出误差
//            for(PointPairList::iterator it=ppl.begin(); it!=ppl.end(); it++)
//            {
//                cout << it->first.x - it->second.x << " " << it->first.y - it->second.y<< endl;
//            }
			
//			double error;
//            if (!method)
//            {
//				cerr << "Nonlinear Optimization" << endl;
//                gettimeofday(&t_start,NULL);
//                error=icpNonlinearStep(tc,ppl);
//                gettimeofday(&t_end,NULL);
//                cerr <<"Nonlinear Time:"<<((t_end.tv_sec - t_start.tv_sec)*1000000+(t_end.tv_usec - t_start.tv_usec))<<"us"<<endl;
//            }
//            else
//            {
//				cerr << "Linear Optimization" << endl;
//                gettimeofday(&t_start,NULL);
//                error=icpLinearStep(tc,ppl);
//                gettimeofday(&t_end,NULL);
//                cerr <<"linear Time:"<<((t_end.tv_sec - t_start.tv_sec)*1000000+(t_end.tv_usec - t_start.tv_usec))<<"us"<<endl;
//			}

//			cout << "plot '-'w l, '-' w p, '-' w p" << endl;
//            //把点云做转换 求解出来的转换关系TC是p1到p2的转换。
//            //也就是说p2 = tc*p1;
//			double s=sin(tc.theta), c=cos(tc.theta);
//            for(PointPairList::iterator it=ppl.begin(); it!=ppl.end(); it++)
//            {
//				Point p1(c*it->first.x-s*it->first.y+tc.x,
//                         s*it->first.x+c*it->first.y+tc.y);
//				it->first=p1;
//			}

//            cerr << "\t" << error << " ttot.x=" << ttot.x << " ttot.y=" << ttot.y << " ttot.theta=" << ttot.theta << endl;

//            //更新原始两帧激光点云之间的坐标 这个经过调试确实是对的
//            double tmp_x,tmp_y;
//            tmp_x = ttot.x*cos(tc.theta) - ttot.y*sin(tc.theta) + tc.x;
//            tmp_y = ttot.x*sin(tc.theta) + ttot.y*cos(tc.theta) + tc.y;

//            ttot.x = tmp_x;
//            ttot.y = tmp_y;
//            ttot.theta+=tc.theta;
//            ttot.theta=atan2(sin(ttot.theta), cos(ttot.theta));

//            cerr << "ICP err=" << error << " t.x=" << tc.x << " t.y=" << tc.y << " t.theta=" << tc.theta << endl;
//            cerr << "\t" << error << " ttot.x=" << ttot.x << " ttot.y=" << ttot.y << " ttot.theta=" << ttot.theta << endl;

//            //计算总的误差
//            error=0;
//            s = sin(ttot.theta);
//            c = cos(ttot.theta);
//            for (int i = 0; i < origin_p1.size(); i++)
//            {
//                Point delta(
//                    c*origin_p1[i].x-s*origin_p1[i].y+ttot.x-origin_p2[i].x,
//                    s*origin_p1[i].x+c*origin_p1[i].y+ttot.y-origin_p2[i].y);
//                error+=delta*delta;
//            }
//            cerr<<"SumErr: "<<error<<endl;

////            ttot.x = tc.x*cos(ttot.theta) + tc.y*sin(ttot.theta) + ttot.x;
////            ttot.y = -tc.x*sin(ttot.theta) + tc.y*cos(ttot.theta) + ttot.y;
////            ttot.theta+=tc.theta;
////            ttot.theta=atan2(sin(ttot.theta), cos(ttot.theta));

////            ttot.x+=tc.x;
////            ttot.y+=tc.y;
////            ttot.theta+=tc.theta;
////            ttot.theta=atan2(sin(ttot.theta), cos(ttot.theta));
//		}
//	}
	return 0;
}
