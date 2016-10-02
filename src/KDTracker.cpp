
#include <KDTracker.hpp>

using namespace pcl;
using namespace std;

KDTracker::KDTracker()
{
  hgoal= 275.484;  //I'm guessing a scanning routine would check
  sgoal = .450776; //skin colors to dynamically get these goals
  vgoal = 87.5668;
  threshold = .05; //a percentage from 0 to 1
  tree = boost::shared_ptr<search::Search<PointXYZRGBA> > (new search::KdTree<PointXYZRGBA>);
}

PointCloud<PointXYZRGBA>::Ptr
KDTracker::Execute(const PointCloud<PointXYZRGBA>::Ptr &cloud_in)
{
    IndicesPtr indices (new std::vector <int>);

PassThrough<PointXYZRGBA> pass;
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

    RegionGrowingRGB<PointXYZRGBA> reg;
  reg.setInputCloud (cloud_in);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);
    reg.setMinClusterSize (300);//original 600

    std::vector <PointIndices> clusters;
    reg.extract (clusters);

    PointCloud <PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

    for(int i =0;i < clusters.size();i++)
    {
	    float h_sum;
	    float s_sum;
	    float v_sum;
	    int counter;
	    for(int j=0; j < clusters[i].indices.size(); j++)
	    {
	    	float r;
		    float g;
		    float b;
	        r =(float) cloud_in->points[clusters[i].indices[j]].r;
	        g = (float) cloud_in->points[clusters[i].indices[j]].g;
	        b = (float) cloud_in->points[clusters[i].indices[j]].b;
			float h;
	  		float s;
	  		float v;
	        if(r>= 0 && r <= 255 && g>= 0 && g <= 255 && b>= 0 && b <= 255)
	        {
	        	counter++;
	            RGBToHSV(r,g,b,&h,&s,&v);
	            h_sum += h;
	            s_sum += s;
	            v_sum += v;

	        }
	        
    
	    }
	   	h_sum/=counter;
	    s_sum/=counter;
	    v_sum/=counter;

	    if(h_sum <= hgoal + (hgoal * threshold) || h_sum >=hgoal - (hgoal * threshold))
	    {
	    	if(s_sum <= sgoal + (sgoal * threshold) || s_sum >=sgoal - (sgoal * threshold))
		    {
		    	if(v_sum <= vgoal + (vgoal * threshold) || v_sum >=vgoal - (vgoal * threshold))
			    {
			    	for(int k=0; k < clusters[i].indices.size(); k++)
	    			{
		     ReturnCloud->points.push_back(cloud_in->points[clusters[i].indices[k]]);
	    			}
			    }
		    }
	    }

	    cout<<"hue sum: "<<h_sum<<endl;
	    cout<<"saturation sum: "<<s_sum<<endl;
	    cout<<"lighting sum: "<<v_sum<<endl;
	}
	cout<<"cluster variable size: "<<clusters.size()<<endl;
    for(int i =0; i < clusters.size(); i++)
    {
        cout<<"cluster num of indices: "<<clusters[i].indices.size()<<endl;
    }

  return ReturnCloud;
}

void KDTracker::RGBToHSV(float r, float g, float b, float *h, float *s, float *v)
{
	float max = r;
	if (max < g) max = g;
	if (max < b) max = b;
	float min = r;
	if (min > g) min = g;
	if (min > b) min = b;
	
	/*
	 *	Calculate h
	 */
	
	*h = 0;
	if (max == min) h = 0;
	else if (max == r) {
		*h = 60 * (g - b)/(max - min);
		if (*h < 0) *h += 360;
		if (*h >= 360) *h -= 360;
	} else if (max == g) {
		*h = 60 * (b - r) / (max - min) + 120;
	} else if (max == b) {
		*h = 60 * (r - g) / (max - min) + 240;
	}

	if (max == 0) *s = 0;
	else *s = 1 - (min / max);

	*v = max;
}

Eigen::Affine3f KDTracker::ComputeTransform()
{
  return Eigen::Affine3f::Identity ();
}

void KDTracker::PrintAlgorithm()
{
  
}
