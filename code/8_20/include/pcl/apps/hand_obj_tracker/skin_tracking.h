/*
 * Software License Agreement (BSD License)
 *
 */

#ifndef PCL_SKIN_TRACKING_H_
#define PCL_SKIN_TRACKING_H_

#include <pcl/pcl_base.h>
#include <Eigen/Core>

#define CLOUD_HEIGHT 240
#define CLOUD_WIDTH 320

namespace pcl
{
	template <typename PointT>
	class skin_tracking :	public PCLBase<PointT>
	{
		using PCLBase<PointT>::initCompute;
    using PCLBase<PointT>::deinitCompute;

		public:
			using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;

			typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      
			/** \brief Empty constructor. */
			skin_tracking () : total_sample_number(0),skin_Tmax(0.5),skin_Tmin(0.2),skin_P_s(0),
												sampleFull(false),spacing(3),stored_frames(0),skin_P_s_w(0),
												total_sample_number_w(0)
			{
				for(int i=0;i<64;i++)
					for(int j=0;j<64;j++)
					{
						skin_P_c_s[i][j] = 0;
						skin_P_c_s_w[i][j] = 0;
					}
				for(int i=0;i<5;i++)
					pre_frame[i].reset(new PointCloud);
			}

			/** \brief Empty destructor. */
			virtual ~skin_tracking() {};

			/** \add new samples until full */
			bool 
			addSamples (const PointCloudConstPtr &cloud);

			/** \remove all samples */
			inline void 
			clearAllSamples ()
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
					pre_frame[i].reset(new PointCloud);
			}

			/** \brief test if enough samples are added */
			inline bool 
			isCalibrated () const{ return(sampleFull);}

			/** \test if the input cloud is organized */
			inline bool
			isOrganziedInput () const		{	return((input_->height == 240) && (input_->width == 320));}
			
			/** */
			inline int
			getSampleNumbers () const {return (total_sample_number);}

			/** \find skin cloud */
			void
			identifySkin (PointCloud &cloud_skin, PointCloud &cloud_other);


		protected:
			/** \convert RGB to YUV */
			inline void
			rgb2uv(int R, int G, int B, int & U, int & V)
			{
				U = (-38*R - 74*G + 112*B +128) >> 8;			
				V = (112*R - 94*G -18*B +128) >> 8;
				U = (U+128)/4; V = (V+128)/4;

			}



		protected:
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
			PointCloudPtr pre_frame[5];
			// skin probability threshold
			float skin_Tmax;
			float skin_Tmin;
			bool sampleFull;
			//
			int spacing;
	};

}

#endif  //#ifndef PCL_SKINTRACKING_H_