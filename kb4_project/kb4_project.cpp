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
	//全局坐标系到每一帧雷达坐标系下的位姿txt
    ifstream infile("/home/zy/work/kb4_project/10-31/position.txt");
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
	//投影图片的文件路径
	if(find_file_under_dir("/home/zy/work/kb4_project/10-31/png",FileName))
	{
		for(size_t n = 0; n < FileName.size();n++){
			string filename=FileName.at(n);
			cout<<filename<<endl;
			double t=stod(filename.substr(7,11));
			cout<<t<<endl;
			//运动去畸变
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
				//全局点云PCD路径
				if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zy/work/kb4_project/10-31/globalDeskewedCloud_fusion.pcd", *cloud_c1) == -1) //* 加载点云数据路径 
				{
					PCL_ERROR("Couldn't read file test_pcd.pcd \n");
					system("PAUSE");
				}
					
				Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);
				pcl::transformPointCloud(*cloud_c1, *cloud_c1, transNow.inverse());
				Eigen::Matrix4f m4f_transform;
				//雷达到相机的外参矩阵
				m4f_transform(0,0)=-0.998148 ;          m4f_transform(0,1)=0.0431709 ;       m4f_transform(0,2)=-0.0428488;m4f_transform(0,3)=  0.0945096;
				m4f_transform(1,0)=0.0428639;           m4f_transform(1,1)=-0.000577437 ;	m4f_transform(1,2)=-0.999081;	m4f_transform(1,3)=-0.00757456;
				m4f_transform(2,0)=-0.043156 ;          m4f_transform(2,1)=-0.999068  ;	m4f_transform(2,2)= -0.0012741;m4f_transform(2,3)=-0.0375876;
				m4f_transform(3,0)=0;                   m4f_transform(3,1)=0;                 m4f_transform(3,2)=0;           m4f_transform(3,3)=1;
				Eigen::Transform<float, 3, Eigen::Affine> transNow1 (m4f_transform);
				pcl::transformPointCloud(*cloud_c1, *cloud_c1, transNow1);	
				string root_path="/home/zy/work/kb4_project/10-31/";
				string img_path=root_path.append("png/").append(FileName.at(n));
				cv::Mat img = cv::imread(img_path);
				Mat depth = Mat(img.rows, img.cols, CV_16UC1, Scalar::all(0));
				Mat dp = Mat(img.rows, img.cols, CV_64FC1, Scalar::all(0));
				Mat disparity = Mat(depth.rows, depth.cols, CV_8U, Scalar(0));
				


				pcl::VoxelGrid<pcl::PointXYZ> sor;
				sor.setInputCloud(cloud_c1);
				sor.setLeafSize(0.15,0.15f,0.15f);
				//sor.filter(*cloud_c1);

					
				vector<cv::Point2f> points_2d;
				vector<float> points_distance;
				for (size_t i = 0; i < cloud_c1->points.size(); i++) {
					if (cloud_c1->points[i].z < 0 && abs(cloud_c1->points[i].z/cloud_c1->points[i].y)>0.35 && abs(cloud_c1->points[i].z/cloud_c1->points[i].x)>0.35) //过滤到后方和超过一定高度的点云，只关注与前方目标点云。反射滤过低说明点云可靠性低
					{
						continue;
					}
					Eigen::Vector3d p_3d(cloud_c1->points[i].x,
					cloud_c1->points[i].y,
					cloud_c1->points[i].z);
					double theta = acos(p_3d(2) / p_3d.norm());
					double phi = atan2(p_3d(1), p_3d(0));
					double k2=-0.014816489660016447;
					double k3=-0.0017851230375137116;
					double k4=-0.0007095084933090205;
					double k5=-0.000002237005312283857;
					Eigen::Vector2d p_u = (theta +k2 * theta * theta * theta +k3 * theta * theta * theta * theta * theta +k4 * theta * theta * theta * theta * theta * theta * theta +k5 * theta * theta * theta * theta * theta * theta * theta * theta * theta)* Eigen::Vector2d(cos(phi), sin(phi));
					Eigen::Vector2d p_2d;
					// Apply generalised projection matrix
					p_2d << 324.5066419233805 * p_u(0) + 565.8493570913548,
						 324.46011190583126* p_u(1) +  557.7195416530303;
					//同一像素点保留最近的像素值
					if(depth.at<ushort>(p_2d(1), p_2d(0))!=0 && depth.at<ushort>(p_2d(1), p_2d(0)) > sqrt(cloud_c1->points[i].x * cloud_c1->points[i].x + cloud_c1->points[i].y * cloud_c1->points[i].y + cloud_c1->points[i].z * cloud_c1->points[i].z)){
						depth.at<ushort>(p_2d(1), p_2d(0))=sqrt(cloud_c1->points[i].x * cloud_c1->points[i].x + cloud_c1->points[i].y * cloud_c1->points[i].y + cloud_c1->points[i].z * cloud_c1->points[i].z);
						dp.at<float>(p_2d(1), p_2d(0))=sqrt(cloud_c1->points[i].x * cloud_c1->points[i].x + cloud_c1->points[i].y * cloud_c1->points[i].y + cloud_c1->points[i].z * cloud_c1->points[i].z);
						points_2d.push_back(cv::Point2f(p_2d(0), p_2d(1)));
						points_distance.push_back(sqrt(cloud_c1->points[i].x * cloud_c1->points[i].x + cloud_c1->points[i].y * cloud_c1->points[i].y + cloud_c1->points[i].z * cloud_c1->points[i].z));
					}
					else if(depth.at<ushort>(p_2d(1), p_2d(0))==0){
						depth.at<ushort>(p_2d(1), p_2d(0))=sqrt(cloud_c1->points[i].x * cloud_c1->points[i].x + cloud_c1->points[i].y * cloud_c1->points[i].y + cloud_c1->points[i].z * cloud_c1->points[i].z);
						dp.at<float>(p_2d(1), p_2d(0))=sqrt(cloud_c1->points[i].x * cloud_c1->points[i].x + cloud_c1->points[i].y * cloud_c1->points[i].y + cloud_c1->points[i].z * cloud_c1->points[i].z);
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
				fstream f;
				string txt_path="/home/zy/work/kb4_project/10-31/result_fusion/";
				txt_path=txt_path.append(FileName.at(n)).append(".txt");
				f.open(txt_path, ios::out | ios::app);
				for (int i = 0; i < depth.rows; i++)
				{
					for (int j = 0; j < depth.cols; j++)
					{
						disparity.ptr<uchar>(i)[j] = 324.5066419233805*0.1115/depth.ptr<ushort>(i)[j];
						f<<dp.ptr<float>(i)[j]<<" ";
					}
				}
				//投影图路径
				string project_path="/home/zy/work/kb4_project/10-31/result_fusion/img_";
				project_path=project_path.append(FileName.at(n));
				//深度图路径
				string depth_path="/home/zy/work/kb4_project/10-31/result_fusion/dep_";
				depth_path=depth_path.append(FileName.at(n));
				//视差图路径
				string dis_path="/home/zy/work/kb4_project/10-31/result_fusion/dis_";
				dis_path=dis_path.append(FileName.at(n));
				cv::imwrite(project_path, img);
				cv::imwrite(depth_path, depth);
				cv::imwrite(dis_path, disparity);
			}
		}				
	}		
}


