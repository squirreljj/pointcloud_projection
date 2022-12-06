#include <iostream>
#include <sys/stat.h>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/3dsc.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/icp.h>
#include <chrono>
#include <math.h> 
// opencv library
#include <opencv2/opencv.hpp>
#include"string.h"
#include<iostream>
#include<sstream>
#include<fstream>
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud1;

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace pcl;
/* 批量读参数 */

//设置标定参数
void distortion(Eigen::Vector2d& p_u, Eigen::Vector2d& d_u)
{
	//distortion_coeffs: [0.16027820518401234, -0.47672871493156865, -0.001875326544400294, -0.0004745820898401468]
	double k1 = 0.16027820518401234;
	double k2 = -0.47672871493156865;
	double p1 = -0.001875326544400294;
	double p2 = -0.0004745820898401468;

	double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

	mx2_u = p_u(0) * p_u(0);
	my2_u = p_u(1) * p_u(1);
	mxy_u = p_u(0) * p_u(1);
	rho2_u = mx2_u + my2_u;
	rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
	d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
		p_u(1)* rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void getColor(float p, float np, float& r, float& g, float& b)
{
	float inc = 6.0 / np;
	float x = p * inc;
	r = 0.0f; g = 0.0f; b = 0.0f;
	if ((0 <= x && x <= 1) || (5 <= x && x <= 6)) r = 1.0f;
	else if (4 <= x && x <= 5) r = x - 4;
	else if (1 <= x && x <= 2) r = 1.0f - (x - 1);

	if (1 <= x && x <= 3) g = 1.0f;
	else if (0 <= x && x <= 1) g = x - 0;
	else if (3 <= x && x <= 4) g = 1.0f - (x - 3);

	if (3 <= x && x <= 5) b = 1.0f;
	else if (2 <= x && x <= 3) b = x - 2;
	else if (5 <= x && x <= 6) b = 1.0f - (x - 5);
	r *= 255.0;
	g *= 255.0;
	b *= 255.0;
}
void Stringsplit(const string& str, const string& splits, vector<string>& res)
{
    if (str == "")		return;
    //在字符串末尾也加入分隔符，方便截取最后一段
    string strs = str + splits;
    size_t pos = strs.find(splits);
    int step = splits.size();

    // 若找不到内容则字符串搜索函数返回 npos
    while (pos != strs.npos)
    {
        string temp = strs.substr(0, pos);
        res.push_back(temp);
        //去掉已分割的字符串,在剩下的字符串中进行分割
        strs = strs.substr(pos + step, strs.size());
        pos = strs.find(splits);
    }
}

bool find_file_under_dir(const string &AbsFilePath,vector<string> &FileName)
{
	DIR *dir;
	struct dirent *ptr;
	if(!(dir = opendir(AbsFilePath.c_str())))
	{
		cout << "current dir isn't exit" << endl;
		return false;
	}
	while((ptr = readdir(dir)) != 0)
	{
		if(strcmp(ptr->d_name,".") == 0 || strcmp(ptr->d_name,"..") == 0)
		{
			continue;
		}
		FileName.push_back(string(ptr->d_name));
	}
	closedir(dir);
	return true;
}

int find_position(vector<string> &dataName,double t)
{
	int data_num=dataName.size()/7;
	for(int i=0;i<data_num;i++)
	{
		if(stod(dataName[i*7+0].substr(7,5))==t){
			cout<<stod(dataName[i*7+0].substr(7,5))<<" "<<t<<endl;		
			return i;
		}
			
	}
	return -1;
}
int main(int argc,char *argv[])  
{ 
	
        
	string str;
        ifstream infile("/home/zy/work/omidirectional/12-2/position.txt");
	//data vector
	vector<string> dataName;
	while(getline(infile,str))
    	{		
		vector<string> list_str;
        	Stringsplit(str," ",list_str);
		dataName.insert(dataName.end(),list_str.begin(),list_str.end());
	}
	cout<<"dataName.size()"<<dataName.size()<<endl;
	int data_num=dataName.size()/7;
	cout<<"data_num"<<data_num<<endl;
	int img_count=0;
	//filename vector
	vector<string> FileName;			
	if(find_file_under_dir("/home/zy/work/omidirectional/12/park2/png",FileName))
	{
		for(size_t n = 0; n < FileName.size();n++){
			string filename=FileName.at(n);
			cout<<filename<<endl;
			double t=stod(filename.substr(7,11));
			cout<<t<<endl;
			int x=find_position(dataName,stod(filename.substr(7,5))-0.1);
			int y=find_position(dataName,stod(filename.substr(7,5))+0.1);
			cout<<"x:"<<x<<" "<<"y:"<<y<<endl;
			if(x!=-1 && y!=-1)
			{
				cout<< ++img_count << endl;
				double t_1 = stod(dataName[x*7+0].substr(7,11));
				double xCur1 = stod(dataName[x*7+4]);
				double yCur1 = stod(dataName[x*7+5]);
				double zCur1 = stod(dataName[x*7+6]);
				double rollCur1 = stod(dataName[x*7+1]); 
				double pitchCur1 = stod(dataName[x*7+2]);
				double yawCur1 = stod(dataName[x*7+3]);
				
				double t_2 = stod(dataName[y*7+0].substr(7,11));
				double xCur2 = stod(dataName[y*7+4]);
				double yCur2 = stod(dataName[y*7+5]);
				double zCur2 = stod(dataName[y*7+6]);
				double rollCur2 = stod(dataName[y*7+1]); 
				double pitchCur2 = stod(dataName[y*7+2]);
				double yawCur2 = stod(dataName[y*7+3]);
			
				double xCur=xCur1+(xCur2-xCur1)*(t-t_1)/(t_2-t_1);
				double yCur=yCur1+(yCur2-yCur1)*(t-t_1)/(t_2-t_1);
				double zCur=zCur1+(zCur2-zCur1)*(t-t_1)/(t_2-t_1);
				double rollCur=rollCur1+(rollCur2-rollCur1)*(t-t_1)/(t_2-t_1);
				double pitchCur=pitchCur1+(pitchCur2-pitchCur1)*(t-t_1)/(t_2-t_1);
				double yawCur=yawCur1+(yawCur2-yawCur1)*(t-t_1)/(t_2-t_1);
				PointCloud1::Ptr cloud_c1(new PointCloud1);//保存彩色相机点云
				//add code
				if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zy/work/omidirectional/12-2/globalDeskewedCloud.pcd", *cloud_c1) == -1) //* 加载点云数据路径 
				{
					PCL_ERROR("Couldn't read file test_pcd.pcd \n");
					system("PAUSE");
				}
					
				Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);
				pcl::transformPointCloud(*cloud_c1, *cloud_c1, transNow.inverse());
				Eigen::Matrix4f m4f_transform;
				//m4f_transform(0,0)=-0.999595;  m4f_transform(0,1)=  0.0284409 ;	m4f_transform(0,2)=0.000392024;	m4f_transform(0,3)=  0.0716666;
				//m4f_transform(1,0)=-0.000309549; m4f_transform(1,1)=0.00290422;	m4f_transform(1,2)=-0.999996;	m4f_transform(1,3)= 0.0419692;
				//m4f_transform(2,0)=-0.028442 	;m4f_transform(2,1)=-0.999591;	m4f_transform(2,2)= -0.00289424;m4f_transform(2,3)=-0.045451;
				//m4f_transform(3,0)=0;m4f_transform(3,1)=0;m4f_transform(3,2)=0;m4f_transform(3,3)=1;
				m4f_transform(0,0)=-0.999563;  m4f_transform(0,1)=  0.0292969  ;	m4f_transform(0,2)=0.00383952;	m4f_transform(0,3)=  0.0736104;
				m4f_transform(1,0)=-0.00377552; m4f_transform(1,1)=0.00223974;	m4f_transform(1,2)=-0.99999;	m4f_transform(1,3)= 0.0422151;
				m4f_transform(2,0)=-0.0293053 	;m4f_transform(2,1)=-0.999568;	m4f_transform(2,2)= -0.00212815;m4f_transform(2,3)=-0.0421832;
				m4f_transform(3,0)=0;m4f_transform(3,1)=0;m4f_transform(3,2)=0;m4f_transform(3,3)=1;
				Eigen::Transform<float, 3, Eigen::Affine> transNow1 (m4f_transform);
				pcl::transformPointCloud(*cloud_c1, *cloud_c1, transNow1);	
				string root_path="/home/zy/work/omidirectional/9-14/louxia1/";
				string img_path=root_path.append("png/").append(FileName.at(n));
				cv::Mat img = cv::imread(img_path);
				Mat depth = Mat(img.rows, img.cols, CV_16UC1, Scalar::all(0));
				Mat disparity = Mat(depth.rows, depth.cols, CV_8U, Scalar(0));
				pcl::VoxelGrid<pcl::PointXYZ> sor;
				sor.setInputCloud(cloud_c1);
				sor.setLeafSize(0.15f, 0.15f, 0.15f);
				//sor.filter(*cloud_c1);

					
				vector<cv::Point2f> points_2d;
				vector<float> points_distance;
				for (size_t i = 0; i < cloud_c1->points.size(); i++) {
					if (cloud_c1->points[i].z < 0 && abs(cloud_c1->points[i].z/cloud_c1->points[i].y)>1.32 &&abs(cloud_c1->points[i].z/cloud_c1->points[i].x)>1.32) //过滤到后方和超过一定高度的点云，只关注与前方目标点云。反射滤过低说明点云可靠性低
					{
						continue;
					}
					Eigen::Vector3d p_3d(cloud_c1->points[i].x,
							cloud_c1->points[i].y,
							cloud_c1->points[i].z);
					Eigen::Vector2d p_u, p_d;
					// Project points to the normalised plane
					double z = p_3d(2) + 1.9493221932722606 * p_3d.norm();
					p_u << p_3d(0) / z, p_3d(1) / z;
					// Apply distortion
					Eigen::Vector2d d_u;
					distortion(p_u, d_u);
					p_d = p_u + d_u;
					/*projection_parameters:
					gamma1: 669.8940458885896
					gamma2: 669.1450614220616
					u0: 377.9459252967363
					v0: 279.63655686698144*/
					//intrinsics: [1.9493221932722606, 610.1324520466419, 609.1651175716506, 507.36820081731776, 381.92387752189603]
					Eigen::Vector2d p_2d;
					p_2d << 610.1324520466419 * p_d(0) + 507.36820081731776,
							609.1651175716506 * p_d(1) + 381.92387752189603;
					//同一像素点保留最近的像素值
					if(depth.at<ushort>(p_2d(1), p_2d(0))!=0 && depth.at<ushort>(p_2d(1), p_2d(0)) > sqrt(cloud_c1->points[i].x * cloud_c1->points[i].x + cloud_c1->points[i].y * cloud_c1->points[i].y + cloud_c1->points[i].z * cloud_c1->points[i].z)){
						depth.at<ushort>(p_2d(1), p_2d(0))=sqrt(cloud_c1->points[i].x * cloud_c1->points[i].x + cloud_c1->points[i].y * cloud_c1->points[i].y + cloud_c1->points[i].z * cloud_c1->points[i].z);
						points_2d.push_back(cv::Point2f(p_2d(0), p_2d(1)));
						points_distance.push_back(sqrt(cloud_c1->points[i].x * cloud_c1->points[i].x + cloud_c1->points[i].y * cloud_c1->points[i].y + cloud_c1->points[i].z * cloud_c1->points[i].z));			
					}
					else if(depth.at<ushort>(p_2d(1), p_2d(0))==0){
						depth.at<ushort>(p_2d(1), p_2d(0))=sqrt(cloud_c1->points[i].x * cloud_c1->points[i].x + cloud_c1->points[i].y * cloud_c1->points[i].y + cloud_c1->points[i].z * cloud_c1->points[i].z);
						points_2d.push_back(cv::Point2f(p_2d(0), p_2d(1)));
						points_distance.push_back(sqrt(cloud_c1->points[i].x * cloud_c1->points[i].x + cloud_c1->points[i].y * cloud_c1->points[i].y + cloud_c1->points[i].z * cloud_c1->points[i].z));							
					}
					

				}
				for (int i = 0; i < (int)points_2d.size(); ++i)
				{
					float r, g, b;
					getColor(points_distance[i], 50.0, r, g, b);
					cv::circle(img, points_2d[i], 0, cv::Scalar(r, g, b), 0.1);
				}
				for (int i = 0; i < depth.rows; i++)
				{
					for (int j = 0; j < depth.cols; j++)
					{
						disparity.ptr<uchar>(i)[j] = 610.1324520466419*0.115/depth.ptr<ushort>(i)[j];
					}
				}
				//add code
				string project_path="/home/zy/work/omidirectional/12-2/louxia1/result_qujibian2/img_";
				project_path=project_path.append(FileName.at(n));

				string depth_path="/home/zy/work/omidirectional/12-2/louxia1/result_qujibian2/dep_";
				depth_path=depth_path.append(FileName.at(n));

				string dis_path="/home/zy/work/omidirectional/12-2/louxia1/result_qujibian2/dis_";
				dis_path=dis_path.append(FileName.at(n));
				cv::imwrite(project_path, img);
				cv::imwrite(depth_path, depth);
				cv::imwrite(dis_path, disparity);
			}
		}				
	}		
}


