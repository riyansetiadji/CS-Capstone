#ifndef SKIN_FILTER_H
#define SKIN_FILTER_H

#include <Algorithm.hpp>

//We may wish to change all these inline functions to regular
//ones and put definitions in the .cpp file for clarity
//More likely we will do everything to never have to worry
//about this code again

class SkinFilter : public Algorithm
{
public:
  // UV skin color probability, P(c|s) 
  int skin_P_c_s[64][64];
  int skin_P_c_s_w[64][64];
  // UV skin color probability, P(s)
  int skin_P_s;
  int skin_P_s_w;
  // number of skin samples 
  int total_sample_number;
  int total_sample_number_w;
  int stored_frames;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pre_frame[5];
  // skin probability threshold
  float skin_Tmax;
  float skin_Tmin;
  bool sampleFull;
  // for finding surrounding points
  int spacing;

  // hypothesis
  bool Hypo[2];

  SkinFilter();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
  Execute(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);
  Eigen::Affine3f ComputeTransform();

  void PrintAlgorithm();

  /** \add new samples until full */
  bool addSamples (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in);

  /** \remove all samples */
  inline void clearAllSamples ()
  {
    sampleFull = false;
    total_sample_number = 0; skin_P_s = 0; skin_P_s_w = 0;stored_frames = 0;
    total_sample_number_w = 0;
    for(int i=0;i<64;i++)
      for(int j=0;j<64;j++)
	{
	  skin_P_c_s[i][j] = 0;
	  skin_P_c_s_w[i][j] = 0;
	}
    for(int i=0;i<5;i++)
      pre_frame[i].reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    Hypo[0]=Hypo[1]=false;
  }

  /** \brief test if enough samples are added */
  inline bool isCalibrated () const{ return(sampleFull);}
		
  /** */
  inline int getSampleNumbers () const 
  { return (total_sample_number); }

  /** \find skin cloud */
  void identifySkin (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_skin,
		     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_other);

protected:
  inline void   /** \convert RGB to YUV */
  rgb2uv(int R, int G, int B, int & U, int & V)
  {
    U = (-38*R - 74*G + 112*B +128) >> 8;			
    V = (112*R - 94*G -18*B +128) >> 8;
    U = (U+128)/4; V = (V+128)/4;
  }
};

#endif
