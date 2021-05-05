#include <iostream>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/keypoints/keypoint.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <vector>
#include <pcl/io/obj_io.h>
#include <ctime>
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/surface/mls.h>

#include<cmath>
#include<ctime>
#include<vector>
#include<string>
#include<algorithm>

//#define E 1.09
//#define E 2.09
#define E 3.09
#define E_difference 1e-3
#define h 220
const int K = 1000;

//const double h = 220.0;
//切片厚度
const double slice_thickness = 120.0;


using namespace std;



int main(int argc, char** argv)
{
	pcl::PCDWriter writer;
	stringstream ss;
	stringstream cc;
	stringstream ff;
	stringstream kk;
	stringstream ee;
	stringstream ss_normals;
	stringstream cc_normals;
	stringstream ff_normals;
	stringstream kk_normals;
	vector<double> every_slice_z; //存储每一个切片的基底坐标

	

	size_t grid_dimension;
	cout << "输入网格的维度: " << endl;
	cin >> grid_dimension;


	vector< vector< pair< double, double > > > v(grid_dimension);
	vector< pair<size_t, size_t> > V_grid_index;
	vector<size_t> V_row_cols;
	vector<pcl::PointXYZ> V_centroid;
	for (size_t i = 0; i != grid_dimension; ++i)
	{
		v[i].resize(grid_dimension);
	}

	pcl::PointCloud<pcl::PointXYZ> cloud0;
	pcl::PointCloud<pcl::PointXYZ> cloud1;
	pcl::PointCloud<pcl::PointXYZ> cloud2;
	pcl::PointCloud<pcl::PointXYZ> cloud3;
	pcl::PointCloud<pcl::PointXYZ> cloud4;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);             //原始点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);        //切片点云中 所有的点
	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_path(new pcl::PointCloud<pcl::PointXYZ>);        //各个切片上的路径点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_path_all(new pcl::PointCloud<pcl::PointXYZ>);    //所有路径点的集合


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centroid(new pcl::PointCloud<pcl::PointXYZ>);    //存储网格点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_paintingpath1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_paintingpath2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZ>);


	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_paintingpath1(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_paintingpath2(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_path_all(new pcl::PointCloud<pcl::Normal>);



	if (pcl::io::loadPCDFile("/home/zhangeaky/final.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file!");
		return -1;
	}
	

	
	int layer_number;
	double side_width;
	double layer_tempz;
	double Max_x, Min_x, Max_y, Min_y, Max_z, Min_z, Dvalue_x, DValue_y, Dvalue_z;
	vector<double> X_temp_all;
	vector<double> Y_temp_all;
	vector<double> Z_temp_all;
	

	/* 遍历点云中的点 将XYZ 的值分别存储 */
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		X_temp_all.push_back(cloud->points[i].x);
		Y_temp_all.push_back(cloud->points[i].y);
		Z_temp_all.push_back(cloud->points[i].z);
	}

	Max_x = *max_element(X_temp_all.begin(), X_temp_all.end());
	Min_x = *min_element(X_temp_all.begin(), X_temp_all.end());
	Max_y = *max_element(Y_temp_all.begin(), Y_temp_all.end());
	Min_y = *min_element(Y_temp_all.begin(), Y_temp_all.end());
	Max_z = *max_element(Z_temp_all.begin(), Z_temp_all.end());
	Min_z = *min_element(Z_temp_all.begin(), Z_temp_all.end());

	Dvalue_x = Max_x - Min_x;
	DValue_y = Max_y - Min_y;
	Dvalue_z = Max_z - Min_z;
	layer_number = Dvalue_z / slice_thickness;     //切片的最初始厚度为 70.5mm
	side_width = (Dvalue_z - slice_thickness * layer_number) / 2;//带宽
	layer_tempz = Min_z + side_width;//基底 + 增量 当前切片坐标
	

	cout << "X最大值: "<<Max_x << " " << Min_x << " " << Max_y << " " << Min_y << " " << Max_z << " " << Min_z << endl;
	cout << "lay_number: " << layer_number << " side_width: " << side_width << " layer_tempz: " << layer_tempz << endl;
	

	//temp用于存储每一层切片Z值坐标的向量
	//
	for (size_t i = 0; i < layer_number + 1; i++){
		every_slice_z.push_back(layer_tempz);
		layer_tempz += slice_thickness;
	}
	cout << "切片个数: " << every_slice_z.size() <<endl;

	cout << "every_slice_z: " << endl;
	for (size_t i = 0; i != every_slice_z.size(); ++i){
		cout << every_slice_z[i] << " " << endl;
	}
	cout << endl;
	

	//主循环
	for (size_t slice = 1; slice != every_slice_z.size(); ++slice){
		cout << every_slice_z[slice] << endl;
		vector<double> Y_temp_1;
		vector<double> Y_temp_2;
		for (size_t j = 0; j != cloud->points.size(); ++j){
			if (cloud->points[j].z > every_slice_z[slice - 1] - E && cloud->points[j].z < every_slice_z[slice - 1] + E)
			{
				Y_temp_1.push_back(cloud->points[j].y);
			}
			if (cloud->points[j].z > every_slice_z[slice] - E && cloud->points[j].z < every_slice_z[slice] + E)
			{
				Y_temp_2.push_back(cloud->points[j].y);
			}
		}

		double Max_y_delta1, Min_y_delta1, Dvalue_y_delta1;
		double Max_y_delta2, Min_y_delta2, Dvalue_y_delta2;

		Max_y_delta1 = *max_element(Y_temp_1.begin(), Y_temp_1.end());
		Min_y_delta1 = *min_element(Y_temp_1.begin(), Y_temp_1.end());
		Max_y_delta2 = *max_element(Y_temp_2.begin(), Y_temp_2.end());
		Min_y_delta2 = *min_element(Y_temp_2.begin(), Y_temp_2.end());

		Dvalue_y_delta1 = Max_y_delta1 - Min_y_delta1;
		Dvalue_y_delta2 = Max_y_delta2 - Min_y_delta2;
		cout << "第一块切片的Y轴最大值" << Max_y_delta1 << "第一块切片的Y轴最小值" << Min_y_delta1 << "第一块切片在Y方向上的长度" << Dvalue_y_delta1 << endl;
		cout << "第二块切片的Y轴最大值" << Max_y_delta2 << "第二块切片的Y轴最小值" << Min_y_delta2 << "第二块切片的Y轴方向的长度" << Dvalue_y_delta2 << endl;

		double dy = (((Max_y_delta1 > Max_y_delta2) ? Max_y_delta1 : Max_y_delta2) - ((Min_y_delta1 < Min_y_delta2) ? Min_y_delta1 : Min_y_delta2)) / (grid_dimension - 1);  //Y��������Ŀ���
		double dz = slice_thickness / (grid_dimension - 1);   //z��������Ŀ���

		for (size_t a = 0; a != grid_dimension; ++a)
		{
			for (size_t b = 0; b != grid_dimension; ++b)
			{
				v[a][b].first = every_slice_z[slice - 1] + b * dz;
				v[a][b].second = ((Max_y_delta1 > Max_y_delta2) ? Max_y_delta1 : Max_y_delta2) - a * dy;
			}
		}


		for (size_t a = 0; a != grid_dimension; ++a)
		{
			int k = 0;
			for (size_t b = 0; b != grid_dimension; ++b)
			{
				for (size_t i = 0; i != cloud->points.size(); ++i)
				{
					if (cloud->points[i].z > v[a][b].first - E && cloud->points[i].z < v[a][b].first + E
						&& cloud->points[i].y > v[a][b].second - E && cloud->points[i].y < v[a][b].second + E)
					{
						pcl::PointXYZ point;
						point.x = cloud->points[i].x;
						point.y = v[a][b].second;
						point.z = v[a][b].first;
						cloud_for->points.push_back(point);  //存储每一个网格内的所有点云
					}
				}

				if (cloud_for->points.size() == 0)
				{
					continue;
				}

				else if (cloud_for->points.size() != 0)
				{
					++k;  

					pair<size_t, size_t> p_index;
					p_index.first = a;
					p_index.second = b;
					V_grid_index.push_back(p_index);   //存储所有网格点的索引

													   //��ʼ������
					cloud_for->height = 1;
					cloud_for->width = cloud_for->points.size();
					cloud_for->points.resize(cloud_for->width * cloud_for->height);

					//�������cloud_temp������
					pcl::PointXYZ centroid;
					double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
					for (size_t j = 0; j != cloud_for->points.size(); ++j)
					{
						sum_x += cloud_for->points[j].x;
						//sum_y += v[a][b].second;
						//sum_z += v[a][b].first;
					}
					centroid.x = sum_x / cloud_for->points.size();
					centroid.y = v[a][b].second;
					centroid.z = v[a][b].first;

					V_centroid.push_back(centroid);     //�洢�������ĵ�����
					cloud_centroid->points.push_back(centroid);  //�������������ĵ�����

					cloud_for->clear();
				}
			}
			V_row_cols.push_back(k);
		}

		//��ʼ�����ĵ���
		cloud_centroid->height = 1;
		cloud_centroid->width = cloud_centroid->points.size();
		cloud_centroid->points.resize(cloud_centroid->height * cloud_centroid->width);

		//����ÿ�������ķ�����
		for (size_t i = 0; i != V_centroid.size(); ++i)
		{
			vector<int> pointIdxNKNSearch(K);
			vector<float> pointNKNSquaredDistance(K);

			Eigen::Vector4f plane_parameters;
			float curvature;

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud(cloud);

			pcl::Normal normal;

			cloud0 = *cloud;

			kdtree.nearestKSearch(V_centroid[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);
			pcl::computePointNormal(cloud0, pointIdxNKNSearch, plane_parameters, curvature);

			if (plane_parameters[0] <= 0)
			{
				normal.normal_x = plane_parameters[0];
				normal.normal_y = plane_parameters[1];
				normal.normal_z = plane_parameters[2];
				normals->points.push_back(normal);
			}
			else if (plane_parameters[0] > 0)
			{
				normal.normal_x = -plane_parameters[0];
				normal.normal_y = -plane_parameters[1];
				normal.normal_z = -plane_parameters[2];
				normals->points.push_back(normal);
			}
			
		}

		//��ʼ��ÿ������ķ�����
		normals->height = 1;
		normals->width = normals->points.size();
		normals->points.resize(normals->height * normals->width);


		pcl::PointXYZ Point1_Max;
		pcl::PointXYZ Point1_Min;
		pcl::PointXYZ Point2_Max;
		pcl::PointXYZ Point2_Min;
		pcl::Normal normal_Point1_Max;
		pcl::Normal normal_Point1_Min;
		pcl::Normal normal_Point2_Max;
		pcl::Normal normal_Point2_Min;

		//��һ�����
		if (Max_y_delta1 < Max_y_delta2 && Min_y_delta1 > Min_y_delta2)
		{
			double Sum_Point1_Max_x = 0.0;
			double Sum_Point1_Min_x = 0.0;
			size_t count_Point1_Max = 0;
			size_t count_Point1_Min = 0;

			for (size_t i = 0; i != cloud->points.size(); ++i)
			{
				if (cloud->points[i].z > every_slice_z[slice - 1] - E && cloud->points[i].z < every_slice_z[slice - 1] + E && cloud->points[i].y > Max_y_delta1 - E)
				{
					Sum_Point1_Max_x += cloud->points[i].x;
					++count_Point1_Max;
				}
				if (cloud->points[i].z > every_slice_z[slice - 1] - E && cloud->points[i].z < every_slice_z[slice - 1] + E && cloud->points[i].y < Min_y_delta1 + E)
				{
					Sum_Point1_Min_x += cloud->points[i].x;
					++count_Point1_Min;
				}
			}

			//cout << count_Point1_Max << " " << count_Point1_Min << endl;
			//�ж�һ��������Χ�ڵĵ�����Ŀ�Ƿ�Ϊ0
			if (count_Point1_Max == 0 || count_Point1_Min == 0)
			{
				cout << "������ƬY���ֵ����Сֵʱ�����������������ķ�Χ����" << endl;
				system("pause");
			}

			Point1_Max.x = Sum_Point1_Max_x / count_Point1_Max;
			Point1_Max.y = Max_y_delta1;
			Point1_Max.z = every_slice_z[slice - 1];
			Point1_Min.x = Sum_Point1_Min_x / count_Point1_Min;
			Point1_Min.y = Min_y_delta1;
			Point1_Min.z = every_slice_z[slice - 1];

			//����Point1_Max��Point1_Min���ķ�������С
			vector<int> pointIdxNKNSearch_Point1_Max(K);
			vector<float> pointNKNSquaredDistance_Point1_Max(K);

			vector<int> pointIdxNKNSearch_Point1_Min(K);
			vector<float> pointNKNSquaredDistance_Point1_Min(K);

			Eigen::Vector4f plane_parameters_Point1_Max;
			float curvature_Point1_Max;

			Eigen::Vector4f plane_parameters_Point1_Min;
			float curvature_Point1_Min;

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point1_Max;
			kdtree_Point1_Max.setInputCloud(cloud);

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point1_Min;
			kdtree_Point1_Min.setInputCloud(cloud);


			cloud1 = *cloud;
			cloud2 = *cloud;

			kdtree_Point1_Max.nearestKSearch(Point1_Max, K, pointIdxNKNSearch_Point1_Max, pointNKNSquaredDistance_Point1_Max);
			pcl::computePointNormal(cloud1, pointIdxNKNSearch_Point1_Max, plane_parameters_Point1_Max, curvature_Point1_Max);

			kdtree_Point1_Min.nearestKSearch(Point1_Min, K, pointIdxNKNSearch_Point1_Min, pointNKNSquaredDistance_Point1_Min);
			pcl::computePointNormal(cloud2, pointIdxNKNSearch_Point1_Min, plane_parameters_Point1_Min, curvature_Point1_Min);



			if (plane_parameters_Point1_Max[0] <= 0)
			{
				normal_Point1_Max.normal_x = plane_parameters_Point1_Max[0];
				normal_Point1_Max.normal_y = plane_parameters_Point1_Max[1];
				normal_Point1_Max.normal_z = plane_parameters_Point1_Max[2];
			}
			else if (plane_parameters_Point1_Max[0] > 0)
			{
				normal_Point1_Max.normal_x = -plane_parameters_Point1_Max[0];
				normal_Point1_Max.normal_y = -plane_parameters_Point1_Max[1];
				normal_Point1_Max.normal_z = -plane_parameters_Point1_Max[2];
			}

			if (plane_parameters_Point1_Min[0] <= 0)
			{
				normal_Point1_Min.normal_x = plane_parameters_Point1_Min[0];
				normal_Point1_Min.normal_y = plane_parameters_Point1_Min[1];
				normal_Point1_Min.normal_z = plane_parameters_Point1_Min[2];
			}
			else if (plane_parameters_Point1_Min[0] > 0)
			{
				normal_Point1_Min.normal_x = -plane_parameters_Point1_Min[0];
				normal_Point1_Min.normal_y = -plane_parameters_Point1_Min[1];
				normal_Point1_Min.normal_z = -plane_parameters_Point1_Min[2];
			}
			


			//����Point2_Max��Point2_Min������λ��
			for (size_t i = 0; i != V_centroid.size(); ++i)
			{

				if (fabs(V_centroid[i].z - every_slice_z[slice]) < E_difference && fabs(V_centroid[i].y - Max_y_delta2) < E_difference)
				{
					Point2_Max.x = V_centroid[i].x;
					Point2_Max.y = V_centroid[i].y;
					Point2_Max.z = V_centroid[i].z;
				}
				if (fabs(V_centroid[i].z - every_slice_z[slice]) < E_difference && fabs(V_centroid[i].y - Min_y_delta2) < E_difference)
				{
					Point2_Min.x = V_centroid[i].x;
					Point2_Min.y = V_centroid[i].y;
					Point2_Min.z = V_centroid[i].z;
				}
			}

			//����Point2_Max��Point2_Min�ķ�����
			vector<int> pointIdxNKNSearch_Point2_Max(K);
			vector<float> pointNKNSquaredDistance_Point2_Max(K);

			vector<int> pointIdxNKNSearch_Point2_Min(K);
			vector<float> pointNKNSquaredDistance_Point2_Min(K);

			Eigen::Vector4f plane_parameters_Point2_Max;
			float curvature_Point2_Max;

			Eigen::Vector4f plane_parameters_Point2_Min;
			float curvature_Point2_Min;

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point2_Max;
			kdtree_Point2_Max.setInputCloud(cloud);

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point2_Min;
			kdtree_Point2_Min.setInputCloud(cloud);


			cloud3 = *cloud;
			cloud4 = *cloud;

			kdtree_Point2_Max.nearestKSearch(Point2_Max, K, pointIdxNKNSearch_Point2_Max, pointNKNSquaredDistance_Point2_Max);
			pcl::computePointNormal(cloud3, pointIdxNKNSearch_Point2_Max, plane_parameters_Point2_Max, curvature_Point2_Max);

			kdtree_Point2_Min.nearestKSearch(Point2_Min, K, pointIdxNKNSearch_Point2_Min, pointNKNSquaredDistance_Point2_Min);
			pcl::computePointNormal(cloud4, pointIdxNKNSearch_Point2_Min, plane_parameters_Point2_Min, curvature_Point2_Min);



			if (plane_parameters_Point2_Max[0] <= 0)
			{
				normal_Point2_Max.normal_x = plane_parameters_Point2_Max[0];
				normal_Point2_Max.normal_y = plane_parameters_Point2_Max[1];
				normal_Point2_Max.normal_z = plane_parameters_Point2_Max[2];
			}
			else if (plane_parameters_Point2_Max[0] > 0)
			{
				normal_Point2_Max.normal_x = -plane_parameters_Point2_Max[0];
				normal_Point2_Max.normal_y = -plane_parameters_Point2_Max[1];
				normal_Point2_Max.normal_z = -plane_parameters_Point2_Max[2];
			}

			if (plane_parameters_Point2_Min[0] <= 0)
			{
				normal_Point2_Min.normal_x = plane_parameters_Point2_Min[0];
				normal_Point2_Min.normal_y = plane_parameters_Point2_Min[1];
				normal_Point2_Min.normal_z = plane_parameters_Point2_Min[2];
			}
			else if (plane_parameters_Point2_Min[0] > 0)
			{
				normal_Point2_Min.normal_x = -plane_parameters_Point2_Min[0];
				normal_Point2_Min.normal_y = -plane_parameters_Point2_Min[1];
				normal_Point2_Min.normal_z = -plane_parameters_Point2_Min[2];
			}
			
		}

		//�ڶ������
		if (Max_y_delta1 > Max_y_delta2 && Min_y_delta1 < Min_y_delta2)
		{
			//����Point2_Max��Point2_Min������λ��
			double Sum_Point2_Max_x = 0.0;
			double Sum_Point2_Min_x = 0.0;
			size_t count_Point2_Max = 0;
			size_t count_Point2_Min = 0;

			for (size_t i = 0; i != cloud->points.size(); ++i)
			{
				if (cloud->points[i].z > every_slice_z[slice] - E && cloud->points[i].z < every_slice_z[slice] + E && cloud->points[i].y > Max_y_delta2 - E)
				{
					Sum_Point2_Max_x += cloud->points[i].x;
					++count_Point2_Max;
				}
				if (cloud->points[i].z > every_slice_z[slice] - E && cloud->points[i].z < every_slice_z[slice] + E && cloud->points[i].y < Min_y_delta2 + E)
				{
					Sum_Point2_Min_x += cloud->points[i].x;
					++count_Point2_Min;
				}
			}

			//�ж�һ��������Χ�ڵĵ�����Ŀ�Ƿ�Ϊ0
			if (count_Point2_Max == 0 || count_Point2_Min == 0)
			{
				cout << "������ƬY���ֵ����Сֵʱ�����������������ķ�Χ����" << endl;
				system("pause");
			}

			Point2_Max.x = Sum_Point2_Max_x / count_Point2_Max;
			Point2_Max.y = Max_y_delta2;
			Point2_Max.z = every_slice_z[slice];
			Point2_Min.x = Sum_Point2_Min_x / count_Point2_Min;
			Point2_Min.y = Min_y_delta2;
			Point2_Min.z = every_slice_z[slice];


			//����Point2_Max��Point2_Min�ķ�����
			vector<int> pointIdxNKNSearch_Point2_Max(K);
			vector<float> pointNKNSquaredDistance_Point2_Max(K);

			vector<int> pointIdxNKNSearch_Point2_Min(K);
			vector<float> pointNKNSquaredDistance_Point2_Min(K);

			Eigen::Vector4f plane_parameters_Point2_Max;
			float curvature_Point2_Max;

			Eigen::Vector4f plane_parameters_Point2_Min;
			float curvature_Point2_Min;

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point2_Max;
			kdtree_Point2_Max.setInputCloud(cloud);

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point2_Min;
			kdtree_Point2_Min.setInputCloud(cloud);


			cloud3 = *cloud;
			cloud4 = *cloud;

			kdtree_Point2_Max.nearestKSearch(Point2_Max, K, pointIdxNKNSearch_Point2_Max, pointNKNSquaredDistance_Point2_Max);
			pcl::computePointNormal(cloud3, pointIdxNKNSearch_Point2_Max, plane_parameters_Point2_Max, curvature_Point2_Max);

			kdtree_Point2_Min.nearestKSearch(Point2_Min, K, pointIdxNKNSearch_Point2_Min, pointNKNSquaredDistance_Point2_Min);
			pcl::computePointNormal(cloud4, pointIdxNKNSearch_Point2_Min, plane_parameters_Point2_Min, curvature_Point2_Min);


			if (plane_parameters_Point2_Max[0] <= 0)
			{
				normal_Point2_Max.normal_x = plane_parameters_Point2_Max[0];
				normal_Point2_Max.normal_y = plane_parameters_Point2_Max[1];
				normal_Point2_Max.normal_z = plane_parameters_Point2_Max[2];
			}
			else if (plane_parameters_Point2_Max[0] > 0)
			{
				normal_Point2_Max.normal_x = -plane_parameters_Point2_Max[0];
				normal_Point2_Max.normal_y = -plane_parameters_Point2_Max[1];
				normal_Point2_Max.normal_z = -plane_parameters_Point2_Max[2];
			}

			if (plane_parameters_Point2_Min[0] <= 0)
			{
				normal_Point2_Min.normal_x = plane_parameters_Point2_Min[0];
				normal_Point2_Min.normal_y = plane_parameters_Point2_Min[1];
				normal_Point2_Min.normal_z = plane_parameters_Point2_Min[2];
			}
			else if (plane_parameters_Point2_Min[0] > 0)
			{
				normal_Point2_Min.normal_x = -plane_parameters_Point2_Min[0];
				normal_Point2_Min.normal_y = -plane_parameters_Point2_Min[1];
				normal_Point2_Min.normal_z = -plane_parameters_Point2_Min[2];
			}
			


			//����Point1_Max��Point1_Min������λ��
			for (size_t i = 0; i != V_centroid.size(); ++i)
			{

				if (fabs(V_centroid[i].z - every_slice_z[slice - 1]) < E_difference && fabs(V_centroid[i].y - Max_y_delta1) < E_difference)
				{
					Point1_Max.x = V_centroid[i].x;
					Point1_Max.y = V_centroid[i].y;
					Point1_Max.z = V_centroid[i].z;
				}
				if (fabs(V_centroid[i].z - every_slice_z[slice - 1]) < E_difference && fabs(V_centroid[i].y - Min_y_delta1) < E_difference)
				{
					Point1_Min.x = V_centroid[i].x;
					Point1_Min.y = V_centroid[i].y;
					Point1_Min.z = V_centroid[i].z;
				}
			}

			//����Point1_Max��Point1_Min�ķ�����
			vector<int> pointIdxNKNSearch_Point1_Max(K);
			vector<float> pointNKNSquaredDistance_Point1_Max(K);

			vector<int> pointIdxNKNSearch_Point1_Min(K);
			vector<float> pointNKNSquaredDistance_Point1_Min(K);

			Eigen::Vector4f plane_parameters_Point1_Max;
			float curvature_Point1_Max;

			Eigen::Vector4f plane_parameters_Point1_Min;
			float curvature_Point1_Min;

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point1_Max;
			kdtree_Point1_Max.setInputCloud(cloud);

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point1_Min;
			kdtree_Point1_Min.setInputCloud(cloud);


			cloud1 = *cloud;
			cloud2 = *cloud;

			kdtree_Point1_Max.nearestKSearch(Point1_Max, K, pointIdxNKNSearch_Point1_Max, pointNKNSquaredDistance_Point1_Max);
			pcl::computePointNormal(cloud1, pointIdxNKNSearch_Point1_Max, plane_parameters_Point1_Max, curvature_Point1_Max);

			kdtree_Point1_Min.nearestKSearch(Point1_Min, K, pointIdxNKNSearch_Point1_Min, pointNKNSquaredDistance_Point1_Min);
			pcl::computePointNormal(cloud2, pointIdxNKNSearch_Point1_Min, plane_parameters_Point1_Min, curvature_Point1_Min);


			if (plane_parameters_Point1_Max[0] <= 0)
			{
				normal_Point1_Max.normal_x = plane_parameters_Point1_Max[0];
				normal_Point1_Max.normal_y = plane_parameters_Point1_Max[1];
				normal_Point1_Max.normal_z = plane_parameters_Point1_Max[2];
			}
			else if (plane_parameters_Point1_Max[0] > 0)
			{
				normal_Point1_Max.normal_x = -plane_parameters_Point1_Max[0];
				normal_Point1_Max.normal_y = -plane_parameters_Point1_Max[1];
				normal_Point1_Max.normal_z = -plane_parameters_Point1_Max[2];
			}

			if (plane_parameters_Point1_Min[0] <= 0)
			{
				normal_Point1_Min.normal_x = plane_parameters_Point1_Min[0];
				normal_Point1_Min.normal_y = plane_parameters_Point1_Min[1];
				normal_Point1_Min.normal_z = plane_parameters_Point1_Min[2];
			}
			else if (plane_parameters_Point1_Min[0] > 0)
			{
				normal_Point1_Min.normal_x = -plane_parameters_Point1_Min[0];
				normal_Point1_Min.normal_y = -plane_parameters_Point1_Min[1];
				normal_Point1_Min.normal_z = -plane_parameters_Point1_Min[2];
			}
			
		}

		//���������
		if (Max_y_delta1 > Max_y_delta2 && Min_y_delta1 > Min_y_delta2)
		{
			//����Point1_Min��Point2_Max������λ��
			double Sum_Point1_Min_x = 0.0;
			double Sum_Point2_Max_x = 0.0;
			size_t count_Point1_Min = 0;
			size_t count_Point2_Max = 0;


			for (size_t i = 0; i != cloud->points.size(); ++i)
			{
				if (cloud->points[i].z > every_slice_z[slice - 1] - E && cloud->points[i].z < every_slice_z[slice - 1] + E && cloud->points[i].y < Min_y_delta1 + E)
				{
					Sum_Point1_Min_x += cloud->points[i].x;
					++count_Point1_Min;
				}
				if (cloud->points[i].z > every_slice_z[slice] - E && cloud->points[i].z < every_slice_z[slice] + E && cloud->points[i].y > Max_y_delta2 - E)
				{
					Sum_Point2_Max_x += cloud->points[i].x;
					++count_Point2_Max;
				}
			}

			//�ж�һ��������Χ�ڵĵ�����Ŀ�Ƿ�Ϊ0
			if (count_Point1_Min == 0 || count_Point2_Max == 0)
			{
				cout << "������ƬY���ֵ����Сֵʱ�����������������ķ�Χ����" << endl;
				system("pause");
			}

			Point1_Min.x = Sum_Point1_Min_x / count_Point1_Min;
			Point1_Min.y = Min_y_delta1;
			Point1_Min.z = every_slice_z[slice - 1];
			Point2_Max.x = Sum_Point2_Max_x / count_Point2_Max;
			Point2_Max.y = Max_y_delta2;
			Point2_Max.z = every_slice_z[slice];

			//����Point1_Min��Point2_Max�ķ�����
			vector<int> pointIdxNKNSearch_Point1_Min(K);
			vector<float> pointNKNSquaredDistance_Point1_Min(K);

			vector<int> pointIdxNKNSearch_Point2_Max(K);
			vector<float> pointNKNSquaredDistance_Point2_Max(K);

			Eigen::Vector4f plane_parameters_Point1_Min;
			float curvature_Point1_Min;

			Eigen::Vector4f plane_parameters_Point2_Max;
			float curvature_Point2_Max;

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point1_Min;
			kdtree_Point1_Min.setInputCloud(cloud);

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point2_Max;
			kdtree_Point2_Max.setInputCloud(cloud);


			cloud2 = *cloud;
			cloud3 = *cloud;

			kdtree_Point1_Min.nearestKSearch(Point1_Min, K, pointIdxNKNSearch_Point1_Min, pointNKNSquaredDistance_Point1_Min);
			pcl::computePointNormal(cloud2, pointIdxNKNSearch_Point1_Min, plane_parameters_Point1_Min, curvature_Point1_Min);

			kdtree_Point2_Max.nearestKSearch(Point2_Max, K, pointIdxNKNSearch_Point2_Max, pointNKNSquaredDistance_Point2_Max);
			pcl::computePointNormal(cloud3, pointIdxNKNSearch_Point2_Max, plane_parameters_Point2_Max, curvature_Point2_Max);


			if (plane_parameters_Point1_Min[0] <= 0)
			{
				normal_Point1_Min.normal_x = plane_parameters_Point1_Min[0];
				normal_Point1_Min.normal_y = plane_parameters_Point1_Min[1];
				normal_Point1_Min.normal_z = plane_parameters_Point1_Min[2];
			}
			else if (plane_parameters_Point1_Min[0] > 0)
			{
				normal_Point1_Min.normal_x = -plane_parameters_Point1_Min[0];
				normal_Point1_Min.normal_y = -plane_parameters_Point1_Min[1];
				normal_Point1_Min.normal_z = -plane_parameters_Point1_Min[2];
			}

			if (plane_parameters_Point2_Max[0] <= 0)
			{
				normal_Point2_Max.normal_x = plane_parameters_Point2_Max[0];
				normal_Point2_Max.normal_y = plane_parameters_Point2_Max[1];
				normal_Point2_Max.normal_z = plane_parameters_Point2_Max[2];
			}
			else if (plane_parameters_Point2_Max[0] > 0)
			{
				normal_Point2_Max.normal_x = -plane_parameters_Point2_Max[0];
				normal_Point2_Max.normal_y = -plane_parameters_Point2_Max[1];
				normal_Point2_Max.normal_z = -plane_parameters_Point2_Max[2];
			}
			

			//����Point1_Max��Point2_Min������λ��
			for (size_t i = 0; i != V_centroid.size(); ++i)
			{
				if (fabs(V_centroid[i].z - every_slice_z[slice - 1]) < E_difference && fabs(V_centroid[i].y - Max_y_delta1) < E_difference)
				{
					Point1_Max.x = V_centroid[i].x;
					Point1_Max.y = V_centroid[i].y;
					Point1_Max.z = V_centroid[i].z;
				}
				if (fabs(V_centroid[i].z - every_slice_z[slice]) < E_difference && fabs(V_centroid[i].y - Min_y_delta2) < E_difference)
				{
					Point2_Min.x = V_centroid[i].x;
					Point2_Min.y = V_centroid[i].y;
					Point2_Min.z = V_centroid[i].z;
				}
			}

			//����Point1_Max��Point2_Min�ķ�����
			vector<int> pointIdxNKNSearch_Point1_Max(K);
			vector<float> pointNKNSquaredDistance_Point1_Max(K);

			vector<int> pointIdxNKNSearch_Point2_Min(K);
			vector<float> pointNKNSquaredDistance_Point2_Min(K);

			Eigen::Vector4f plane_parameters_Point1_Max;
			float curvature_Point1_Max;

			Eigen::Vector4f plane_parameters_Point2_Min;
			float curvature_Point2_Min;

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point1_Max;
			kdtree_Point1_Max.setInputCloud(cloud);

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point2_Min;
			kdtree_Point2_Min.setInputCloud(cloud);


			cloud1 = *cloud;
			cloud4 = *cloud;

			kdtree_Point1_Max.nearestKSearch(Point1_Max, K, pointIdxNKNSearch_Point1_Max, pointNKNSquaredDistance_Point1_Max);
			pcl::computePointNormal(cloud1, pointIdxNKNSearch_Point1_Max, plane_parameters_Point1_Max, curvature_Point1_Max);

			kdtree_Point2_Min.nearestKSearch(Point2_Min, K, pointIdxNKNSearch_Point2_Min, pointNKNSquaredDistance_Point2_Min);
			pcl::computePointNormal(cloud4, pointIdxNKNSearch_Point2_Min, plane_parameters_Point2_Min, curvature_Point2_Min);


			if (plane_parameters_Point1_Max[0] <= 0)
			{
				normal_Point1_Max.normal_x = plane_parameters_Point1_Max[0];
				normal_Point1_Max.normal_y = plane_parameters_Point1_Max[1];
				normal_Point1_Max.normal_z = plane_parameters_Point1_Max[2];
			}
			else if (plane_parameters_Point1_Max[0] > 0)
			{
				normal_Point1_Max.normal_x = -plane_parameters_Point1_Max[0];
				normal_Point1_Max.normal_y = -plane_parameters_Point1_Max[1];
				normal_Point1_Max.normal_z = -plane_parameters_Point1_Max[2];
			}

			if (plane_parameters_Point2_Min[0] <= 0)
			{
				normal_Point2_Min.normal_x = plane_parameters_Point2_Min[0];
				normal_Point2_Min.normal_y = plane_parameters_Point2_Min[1];
				normal_Point2_Min.normal_z = plane_parameters_Point2_Min[2];
			}
			else if (plane_parameters_Point2_Min[0] > 0)
			{
				normal_Point2_Min.normal_x = -plane_parameters_Point2_Min[0];
				normal_Point2_Min.normal_y = -plane_parameters_Point2_Min[1];
				normal_Point2_Min.normal_z = -plane_parameters_Point2_Min[2];
			}
			

		}

		//���������
		if (Max_y_delta1 < Max_y_delta2 && Min_y_delta1 < Min_y_delta2)
		{
			//����Point1_Max��Point2_Min��λ������
			double Sum_Point1_Max_x = 0.0;
			double Sum_Point2_Min_x = 0.0;
			size_t count_Point1_Max = 0;
			size_t count_Point2_Min = 0;

			for (size_t i = 0; i != cloud->points.size(); ++i)
			{
				if (cloud->points[i].z > every_slice_z[slice - 1] - E && cloud->points[i].z < every_slice_z[slice - 1] + E && cloud->points[i].y > Max_y_delta1 - E)
				{
					Sum_Point1_Max_x += cloud->points[i].x;
					++count_Point1_Max;
				}
				if (cloud->points[i].z > every_slice_z[slice] - E && cloud->points[i].z < every_slice_z[slice] + E && cloud->points[i].y < Min_y_delta2 + E)
				{
					Sum_Point2_Min_x += cloud->points[i].x;
					++count_Point2_Min;
				}
			}

			//�ж�һ��������Χ�ڵĵ�����Ŀ�Ƿ�Ϊ0
			if (count_Point1_Max == 0 || count_Point2_Min == 0)
			{
				cout << "������ƬY���ֵ����Сֵʱ�����������������ķ�Χ����" << endl;
				system("pause");
			}

			Point1_Max.x = Sum_Point1_Max_x / count_Point1_Max;
			Point1_Max.y = Max_y_delta1;
			Point1_Max.z = every_slice_z[slice - 1];
			Point2_Min.x = Sum_Point2_Min_x / count_Point2_Min;
			Point2_Min.y = Min_y_delta2;
			Point2_Min.z = every_slice_z[slice];

			//����Point1_Max��Point2_Min�ķ�����
			vector<int> pointIdxNKNSearch_Point1_Max(K);
			vector<float> pointNKNSquaredDistance_Point1_Max(K);

			vector<int> pointIdxNKNSearch_Point2_Min(K);
			vector<float> pointNKNSquaredDistance_Point2_Min(K);

			Eigen::Vector4f plane_parameters_Point1_Max;
			float curvature_Point1_Max;

			Eigen::Vector4f plane_parameters_Point2_Min;
			float curvature_Point2_Min;

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point1_Max;
			kdtree_Point1_Max.setInputCloud(cloud);

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point2_Min;
			kdtree_Point2_Min.setInputCloud(cloud);


			cloud1 = *cloud;
			cloud4 = *cloud;

			kdtree_Point1_Max.nearestKSearch(Point1_Max, K, pointIdxNKNSearch_Point1_Max, pointNKNSquaredDistance_Point1_Max);
			pcl::computePointNormal(cloud1, pointIdxNKNSearch_Point1_Max, plane_parameters_Point1_Max, curvature_Point1_Max);

			kdtree_Point2_Min.nearestKSearch(Point2_Min, K, pointIdxNKNSearch_Point2_Min, pointNKNSquaredDistance_Point2_Min);
			pcl::computePointNormal(cloud4, pointIdxNKNSearch_Point2_Min, plane_parameters_Point2_Min, curvature_Point2_Min);


			if (plane_parameters_Point1_Max[0] <= 0)
			{
				normal_Point1_Max.normal_x = plane_parameters_Point1_Max[0];
				normal_Point1_Max.normal_y = plane_parameters_Point1_Max[1];
				normal_Point1_Max.normal_z = plane_parameters_Point1_Max[2];
			}
			else if (plane_parameters_Point1_Max[0] > 0)
			{
				normal_Point1_Max.normal_x = -plane_parameters_Point1_Max[0];
				normal_Point1_Max.normal_y = -plane_parameters_Point1_Max[1];
				normal_Point1_Max.normal_z = -plane_parameters_Point1_Max[2];
			}

			if (plane_parameters_Point2_Min[0] <= 0)
			{
				normal_Point2_Min.normal_x = plane_parameters_Point2_Min[0];
				normal_Point2_Min.normal_y = plane_parameters_Point2_Min[1];
				normal_Point2_Min.normal_z = plane_parameters_Point2_Min[2];
			}
			else if (plane_parameters_Point2_Min[0] > 0)
			{
				normal_Point2_Min.normal_x = -plane_parameters_Point2_Min[0];
				normal_Point2_Min.normal_y = -plane_parameters_Point2_Min[1];
				normal_Point2_Min.normal_z = -plane_parameters_Point2_Min[2];
			}
			

			//����Point1_Min��Point2_Max������λ��
			for (size_t i = 0; i != V_centroid.size(); ++i)
			{
				if (fabs(V_centroid[i].z - every_slice_z[slice - 1]) < E_difference && fabs(V_centroid[i].y - Min_y_delta1) < E_difference)
				{
					Point1_Min.x = V_centroid[i].x;
					Point1_Min.y = V_centroid[i].y;
					Point1_Min.z = V_centroid[i].z;
				}
				if (fabs(V_centroid[i].z - every_slice_z[slice]) < E_difference && fabs(V_centroid[i].y - Max_y_delta2) < E_difference)
				{
					Point2_Max.x = V_centroid[i].x;
					Point2_Max.y = V_centroid[i].y;
					Point2_Max.z = V_centroid[i].z;
				}
			}

			//����Point1_Min��Point2_Max���ķ�����
			vector<int> pointIdxNKNSearch_Point1_Min(K);
			vector<float> pointNKNSquaredDistance_Point1_Min(K);

			vector<int> pointIdxNKNSearch_Point2_Max(K);
			vector<float> pointNKNSquaredDistance_Point2_Max(K);

			Eigen::Vector4f plane_parameters_Point1_Min;
			float curvature_Point1_Min;

			Eigen::Vector4f plane_parameters_Point2_Max;
			float curvature_Point2_Max;

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point1_Min;
			kdtree_Point1_Min.setInputCloud(cloud);

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Point2_Max;
			kdtree_Point2_Max.setInputCloud(cloud);


			cloud2 = *cloud;
			cloud3 = *cloud;

			kdtree_Point1_Min.nearestKSearch(Point1_Min, K, pointIdxNKNSearch_Point1_Min, pointNKNSquaredDistance_Point1_Min);
			pcl::computePointNormal(cloud2, pointIdxNKNSearch_Point1_Min, plane_parameters_Point1_Min, curvature_Point1_Min);

			kdtree_Point2_Max.nearestKSearch(Point2_Max, K, pointIdxNKNSearch_Point2_Max, pointNKNSquaredDistance_Point2_Max);
			pcl::computePointNormal(cloud3, pointIdxNKNSearch_Point2_Max, plane_parameters_Point2_Max, curvature_Point2_Max);


			if (plane_parameters_Point1_Min[0] <= 0)
			{
				normal_Point1_Min.normal_x = plane_parameters_Point1_Min[0];
				normal_Point1_Min.normal_y = plane_parameters_Point1_Min[1];
				normal_Point1_Min.normal_z = plane_parameters_Point1_Min[2];
			}
			else if (plane_parameters_Point1_Min[0] > 0)
			{
				normal_Point1_Min.normal_x = -plane_parameters_Point1_Min[0];
				normal_Point1_Min.normal_y = -plane_parameters_Point1_Min[1];
				normal_Point1_Min.normal_z = -plane_parameters_Point1_Min[2];
			}

			if (plane_parameters_Point2_Max[0] <= 0)
			{
				normal_Point2_Max.normal_x = plane_parameters_Point2_Max[0];
				normal_Point2_Max.normal_y = plane_parameters_Point2_Max[1];
				normal_Point2_Max.normal_z = plane_parameters_Point2_Max[2];
			}
			else if (plane_parameters_Point2_Max[0] > 0)
			{
				normal_Point2_Max.normal_x = -plane_parameters_Point2_Max[0];
				normal_Point2_Max.normal_y = -plane_parameters_Point2_Max[1];
				normal_Point2_Max.normal_z = -plane_parameters_Point2_Max[2];
			}
			
		}



		vector<size_t> V_centroid_paintingpath1;
		vector<size_t> V_centroid_paintingpath2;
		if (Max_y_delta1 < Max_y_delta2 && Min_y_delta1 > Min_y_delta2)
		{
			cloud_paintingpath1->points.push_back(Point1_Max);
		}
		if (Max_y_delta1 > Max_y_delta2 && Min_y_delta1 < Min_y_delta2)
		{
			cloud_paintingpath2->points.push_back(Point2_Max);
		}
		if (Max_y_delta1 > Max_y_delta2 && Min_y_delta1 > Min_y_delta2)
		{
			cloud_paintingpath2->points.push_back(Point2_Max);
		}
		if (Max_y_delta1 < Max_y_delta2 && Min_y_delta1 < Min_y_delta2)
		{
			cloud_paintingpath1->points.push_back(Point1_Max);
		}
		
		for (size_t j = 0; j != V_centroid.size(); ++j)
		{

			if (fabs(V_centroid[j].z - every_slice_z[slice - 1]) < E_difference)
			{
				V_centroid_paintingpath1.push_back(j);
				pcl::PointXYZ point;
				point.x = V_centroid[j].x;
				point.y = V_centroid[j].y;
				point.z = V_centroid[j].z;
				cloud_paintingpath1->points.push_back(point);
			}
			if (fabs(V_centroid[j].z - every_slice_z[slice]) < E_difference)
			{
				V_centroid_paintingpath2.push_back(j);
				pcl::PointXYZ point;
				point.x = V_centroid[j].x;
				point.y = V_centroid[j].y;
				point.z = V_centroid[j].z;
				cloud_paintingpath2->points.push_back(point);
			}
		}
		
		if (Max_y_delta1 < Max_y_delta2 && Min_y_delta1 > Min_y_delta2)
		{
			cloud_paintingpath1->points.push_back(Point1_Min);
		}
		if (Max_y_delta1 > Max_y_delta2 && Min_y_delta1 < Min_y_delta2)
		{
			cloud_paintingpath2->points.push_back(Point2_Min);
		}
		if (Max_y_delta1 > Max_y_delta2 && Min_y_delta1 > Min_y_delta2)
		{
			cloud_paintingpath1->points.push_back(Point1_Min);
		}
		if (Max_y_delta1 < Max_y_delta2 && Min_y_delta1 < Min_y_delta2)
		{
			cloud_paintingpath2->points.push_back(Point2_Min);
		}

		//��ʼ��2���켣�ϵĵ���
		cloud_paintingpath1->height = 1;
		cloud_paintingpath1->width = cloud_paintingpath1->points.size();
		cloud_paintingpath1->points.resize(cloud_paintingpath1->width * cloud_paintingpath1->height);

		cloud_paintingpath2->height = 1;
		cloud_paintingpath2->width = cloud_paintingpath2->points.size();
		cloud_paintingpath2->points.resize(cloud_paintingpath2->width * cloud_paintingpath2->height);


		

		if (Max_y_delta1 < Max_y_delta2 && Min_y_delta1 > Min_y_delta2)
		{
			normals_paintingpath1->points.push_back(normal_Point1_Max);
		}
		if (Max_y_delta1 < Max_y_delta2 && Min_y_delta1 < Min_y_delta2)
		{
			normals_paintingpath1->points.push_back(normal_Point1_Max);
		}
		
		for (size_t j = 0; j != V_centroid_paintingpath1.size(); ++j)
		{
			normals_paintingpath1->points.push_back(normals->points[V_centroid_paintingpath1[j]]);
		}
		
		if (Max_y_delta1 < Max_y_delta2 && Min_y_delta1 > Min_y_delta2)
		{
			normals_paintingpath1->points.push_back(normal_Point1_Min);
		}
		if (Max_y_delta1 > Max_y_delta2 && Min_y_delta1 > Min_y_delta2)
		{
			normals_paintingpath1->points.push_back(normal_Point1_Min);
		}

		normals_paintingpath1->height = 1;
		normals_paintingpath1->width = normals_paintingpath1->points.size();
		normals_paintingpath1->points.resize(normals_paintingpath1->width * normals_paintingpath1->height);

		

		if (Max_y_delta1 > Max_y_delta2 && Min_y_delta1 < Min_y_delta2)
		{
			normals_paintingpath2->points.push_back(normal_Point2_Max);
		}
		if (Max_y_delta1 > Max_y_delta2 && Min_y_delta1 > Min_y_delta2)
		{
			normals_paintingpath2->points.push_back(normal_Point2_Max);
		}
		
		for (size_t j = 0; j != V_centroid_paintingpath2.size(); ++j)
		{
			normals_paintingpath2->points.push_back(normals->points[V_centroid_paintingpath2[j]]);
		}
		
		if (Max_y_delta1 > Max_y_delta2 && Min_y_delta1 < Min_y_delta2)
		{
			normals_paintingpath2->points.push_back(normal_Point2_Min);
		}
		if (Max_y_delta1 < Max_y_delta2 && Min_y_delta1 < Min_y_delta2)
		{
			normals_paintingpath2->points.push_back(normal_Point2_Min);
		}

		normals_paintingpath2->height = 1;
		normals_paintingpath2->width = normals_paintingpath2->points.size();
		normals_paintingpath2->points.resize(normals_paintingpath2->width * normals_paintingpath2->height);

		for (size_t i = 0; i != cloud_paintingpath1->points.size(); ++i)
		{
			pcl::PointXYZ point;
			point.x = cloud_paintingpath1->points[i].x + h * normals_paintingpath1->points[i].normal_x;
			point.y = cloud_paintingpath1->points[i].y + h * normals_paintingpath1->points[i].normal_y;
			point.z = cloud_paintingpath1->points[i].z + h * normals_paintingpath1->points[i].normal_z;
			cloud_path->points.push_back(point);                               //cloud_pathΪ������Ϳ�켣�ϵ�ļ���
			cloud_path_all->points.push_back(point);                           //cloud_path_allΪ��Ϳ�켣�����е�ļ���
			normals_cloud_path_all->points.push_back(normals_paintingpath1->points[i]);
			cloud_temp->points.push_back(cloud_paintingpath1->points[i]);      //cloud_tempΪ��Ƭ�����е�ļ���
		}

		cloud_path->height = 1;
		cloud_path->width = cloud_path->points.size();
		cloud_path->points.resize(cloud_path->height * cloud_path->width);


		


		if (slice != (every_slice_z.size() - 1))
		{
			ss << "D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\point_cloud_slicing\\mianban_" << (slice - 1) << ".pcd";
			cc << "D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\painting_path\\mianban_" << (slice - 1) << ".pcd";
			ss_normals << "D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\normals_path\\normals_mianban_" << (slice - 1) << ".pcd";
			pcl::io::savePCDFile(ss_normals.str(), *normals_paintingpath1);
			pcl::io::savePCDFile(ss.str(), *cloud_paintingpath1);                 //������Ƭ�ϵ�ļ���
			pcl::io::savePCDFile(cc.str(), *cloud_path);
			ss.str("");
			cc.str("");
			ss_normals.str("");
		}

		else if (slice == (every_slice_z.size() - 1))
		{
			for (size_t i = 0; i != cloud_paintingpath2->points.size(); ++i)
			{
				pcl::PointXYZ point;
				point.x = cloud_paintingpath2->points[i].x + h * normals_paintingpath2->points[i].normal_x;
				point.y = cloud_paintingpath2->points[i].y + h * normals_paintingpath2->points[i].normal_y;
				point.z = cloud_paintingpath2->points[i].z + h * normals_paintingpath2->points[i].normal_z;
				cloud_temp1->points.push_back(point);
				cloud_temp->points.push_back(cloud_paintingpath2->points[i]);
				cloud_path_all->points.push_back(point);
				normals_cloud_path_all->points.push_back(normals_paintingpath2->points[i]);
			}
			cloud_temp1->height = 1;
			cloud_temp1->width = cloud_temp1->points.size();
			cloud_temp1->points.resize(cloud_temp1->height * cloud_temp1->width);
			ss << "D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\point_cloud_slicing\\mianban_" << (slice - 1) << ".pcd";
			ff << "D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\point_cloud_slicing\\mianban_" << slice << ".pcd";
			cc << "D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\painting_path\\mianban_" << (slice - 1) << ".pcd";
			kk << "D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\painting_path\\mianban_" << slice << ".pcd";
			ss_normals << "D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\normals_path\\normals_mianban_" << (slice - 1) << ".pcd";
			cc_normals << "D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\normals_path\\normals_mianban_" << slice << ".pcd";
			pcl::io::savePCDFile(cc.str(), *cloud_path);
			pcl::io::savePCDFile(kk.str(), *cloud_temp1);
			pcl::io::savePCDFile(ss.str(), *cloud_paintingpath1);
			pcl::io::savePCDFile(ff.str(), *cloud_paintingpath2);
			pcl::io::savePCDFile(ss_normals.str(), *normals_paintingpath1);
			pcl::io::savePCDFile(cc_normals.str(), *normals_paintingpath2);

			ss.str("");
			cc.str("");
			kk.str("");
			ff.str("");
			ss_normals.str("");
			cc_normals.str("");
		}





		cloud_paintingpath1->clear();
		cloud_paintingpath2->clear();
		normals_paintingpath1->clear();
		normals_paintingpath2->clear();
		cloud_path->clear();
		cloud_centroid->clear();
		cloud_temp1->clear();
		normals->clear();

		V_grid_index.clear();
		V_row_cols.clear();
		V_centroid.clear();

	}

	cloud_path_all->height = 1;
	cloud_path_all->width = cloud_path_all->points.size();
	cloud_path_all->points.resize(cloud_path_all->height * cloud_path_all->width);

	cloud_temp->height = 1;
	cloud_temp->width = cloud_temp->points.size();
	cloud_temp->points.resize(cloud_temp->height * cloud_temp->width);


	normals_cloud_path_all->height = 1;
	normals_cloud_path_all->width = normals_cloud_path_all->points.size();
	normals_cloud_path_all->points.resize(normals_cloud_path_all->height * normals_cloud_path_all->width);




	

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();
	viewer->addCoordinateSystem();

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud(cloud, "cloud");
	viewer->addPointCloud(cloud_temp, "cloud_temp");              //cloud_tempΪ��ƽ���ϵĵ�
	viewer->addPointCloud(cloud_path_all, "cloud_path_all");      //cloud_path_allΪ��Ϳ�켣��
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_temp, normals_cloud_path_all, 1, 10, "normals");


	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "cloud_temp");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "cloud_path_all");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "normals");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_temp");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_path_all");
	


	

	pcl::io::savePCDFile("D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\kobe\\cloud_temp.pcd", *cloud_temp);
	pcl::io::savePCDFile("D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\kobe\\cloud_path_all.pcd", *cloud_path_all);
	pcl::io::savePCDFile("D:\\file\\PointCloudData_mianban\\mianban_temp\\path_generation4\\kobe\\normals_cloud_path_all.pcd", *normals_cloud_path_all);


	while (!viewer->wasStopped())
	{
	    viewer->spinOnce();
	}
	
	system("pause");
	return (0);
}
