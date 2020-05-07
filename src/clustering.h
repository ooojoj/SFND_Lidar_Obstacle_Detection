#ifndef CLUSTERING_H_
#define CLUSTERINH_H_


inline void clusterHelper(int index, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) 
{
	processed[index] = true;
	cluster.push_back(index);

	std::vector<int> nearest = tree->search(points[index], distanceTol);

	for (int id : nearest) {
		if (!processed[id]) {
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
		}
	}
}

inline std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(),false);

	int i=0;
	while (i<points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		} 
		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}	
 
	return clusters;

}

#endif