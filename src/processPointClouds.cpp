// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // voxel grid point reduction and region based filtering
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudROI (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudROI);

  	std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudROI);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
    {
      inliers->indices.push_back(point); 
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudROI);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudROI);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudROI;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  
	// Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud,planeCloud);

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    
	// Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    seg.setOptimizeCoefficients (true);    
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    //Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    
  	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
  	//pcl::PointIndices::Ptr inliers;	
	float x1, x2, x3;
    float y1, y2, y3;
    float z1, z2, z3;  
  	float a, b, c, d;
  	float distance, temp;
  
  	while(maxIterations--)
	{
		//Randomly pick three points
		std::unordered_set<int> inliers;
		while (inliers.size()<3)
		{
			inliers.insert(rand()%(cloud->points.size())); //between zero and number of points in cloud
		}

		auto itr =inliers.begin();
		x1=cloud->points[*itr].x;
		y1=cloud->points[*itr].y;
		z1=cloud->points[*itr].z;
		itr++;
		x2=cloud->points[*itr].x;
		y2=cloud->points[*itr].y;
		z2=cloud->points[*itr].z;
		itr++;
		x3=cloud->points[*itr].x;
		y3=cloud->points[*itr].y;
		z3=cloud->points[*itr].z;
		
		// This iteration's plane is modeled by the equation Ax + By + Cz + D = 0
		
		a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		d = -(a*x1+b*y1+c*z1);
      	temp=sqrt(a*a+b*b+c*c);

		for(int index=0; index < cloud->points.size(); index++)
		{
			//if(inliers.count(index)>0)
			//continue;			

			distance=fabs(a*cloud->points[index].x+b*cloud->points[index].y+c*cloud->points[index].z+d)/temp;

			if(distance<=distanceThreshold)
				inliers.insert(index);
		}
      
		if (inliers.size()>inliersResult.size())
		{
			inliersResult=inliers;
		}
	}
  
    if (inliersResult.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
  
    
    //seprate clouds
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
    
    for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstacleCloud->points.push_back(point);
	}  
      
   	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
      
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();   
  
   	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    // perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
        {
            cloudCluster->points.push_back (cloud->points[index]);
        }
        cloudCluster->width=cloudCluster->points.size();
        cloudCluster->height=1;
        cloudCluster->is_dense=true;
        
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringEuclidean(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();   
  
   	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;   	
  	KdTree* tree = new KdTree;
  
  	std::vector<std::vector<float>> cloud2Vector;
  
    // perform euclidean clustering to group detected obstacles
    for(int i = 0; i < cloud->points.size(); i++)
    {
      std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      tree->insert(point, i); //create a KDtree
      cloud2Vector.push_back(point);      //create vector for Euclidean clustering 
    }
	
   	std::vector<std::vector<int>> results;
  	results = euclideanCluster(cloud2Vector, tree, clusterTolerance);

    for(auto indexResults : results)
    {
        //drop clusters below and above tresholds
      	if(indexResults.size() < minSize || indexResults.size() > maxSize) 
    	{
        	continue; 
    	}
        
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : indexResults)
        {
            cloudCluster->points.push_back (cloud->points[index]);
        }
        cloudCluster->width=cloudCluster->points.size();
        cloudCluster->height=1;
        cloudCluster->is_dense=true;
        
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}



template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}