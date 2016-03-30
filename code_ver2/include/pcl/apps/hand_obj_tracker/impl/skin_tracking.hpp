/*
 * Software License Agreement (BSD License)
 *
 */

#ifndef PCL_IMPL_SKIN_TRACKING_H_
#define PCL_IMPL_SKIN_TRACKING_H_

#include <iostream>
#include <pcl/apps/hand_obj_tracker/skin_tracking.h>

template <typename PointT> bool 
pcl::skin_tracking<PointT>::addSamples (const PointCloudConstPtr &cloud_in)
{
	bool isSuccess = true;
	if (sampleFull)
	{
		isSuccess = false;
	}
	else
	{
		for (size_t i = 0; i < cloud_in->points.size (); ++i)
		{
			int U, V;
			rgb2uv(cloud_in->points[i].r, cloud_in->points[i].g, cloud_in->points[i].b, U, V);
			skin_P_c_s[U][V] ++;
			total_sample_number++;
		}

		if (total_sample_number>5000)
		{
			sampleFull = true;

			skin_P_s = 0;
			for(int i=0; i < 64;i++)
				for(int j=0; j < 64;j++)
				{
					if (skin_P_c_s[i][j] > total_sample_number/50)
						skin_P_s ++;
				}
		}
	}


	return(isSuccess);
}

template <typename PointT> void 
pcl::skin_tracking<PointT>::identifySkin (PointCloud &cloud_skin, PointCloud &cloud_other)
{
			
	// input_ height = 240, width = 320
	if(sampleFull && isOrganziedInput())
	{
		Eigen::MatrixXf P_s_c(320,240);
		Eigen::MatrixXi labels = Eigen::MatrixXi::Zero(320,240);
		bool skinMask[320][240]={false};
		int U, V;
		
		// convert rgb to yuv, calculate skin color probability
		for (size_t i = 0; i < 320; ++i)
			for (size_t j = 0; j < 240; ++j)
			{
				rgb2uv(input_->at(i,j).r, input_->at(i,j).g, input_->at(i,j).b, U, V);
				P_s_c(i,j) = (float)skin_P_c_s[U][V] / (float)total_sample_number * (float)skin_P_s;
				if (total_sample_number_w > 0)
					P_s_c(i,j) = P_s_c(i,j)*0.8 + 0.2*(float)skin_P_c_s_w[U][V] / (float)total_sample_number_w * (float)skin_P_s_w;
			}

		// label skin color seed point
		for (int i = spacing; i < (320-spacing); ++i)
			for (int j = spacing; j < (240-spacing); ++j)
				if ( P_s_c(i,j)> skin_Tmax )
				{
					skinMask[i][j] = true; 
					for (int m = (i-spacing); m<(i+spacing) ; m++)
						for (int n = (j-spacing); n<(j+spacing) ; n++)
							if ( P_s_c(m,n)> skin_Tmin)
								skinMask[m][n] = true;
				}
					
		//for (int i = spacing; i < (320-spacing); ++i)
		//	for (int j = spacing; j < (240-spacing); ++j)
		//		if(skinMask[i][j])
		//			for (int m = (i-spacing); m<(i+spacing) ; m++)
		//				for (int n = (j-spacing); n<(j+spacing) ; n++)
		//					if ( P_s_c(m,n)> skin_Tmin)
		//						skinMask[m][n] = true; 

		int currentLabel = 0;
		int C[5000];
		for (int i =0; i<5000; i++)
			C[i] = i;
		
		for (int i = 2; i < (320-2); ++i)
			for (int j = 2; j < (240-2); ++j)
				if ( skinMask[i][j] )
				{
					int connected = 0;
					int temp_label = 0;
					for (int m = (i-2); m<(i+2) ; m++)
						for (int n = (j-2); n<(j+2) ; n++)
							if ((labels(m,n)>0) && (labels(m,n)!=temp_label) )
							{
								temp_label = labels(m,n);
								connected++;
							}

					if (connected == 0)
					{
						currentLabel++;
						labels(i,j)=currentLabel;
					}
					else
					{
						labels(i,j) = temp_label;
					}

								
					//labels(i,j) = 1;
					for (int m = (i-2); m<(i+2) ; m++)
						for (int n = (j-2); n<(j+2) ; n++)
						{
							if ((C[labels(m,n)]!=C[temp_label]) && (connected >1) && (labels(m,n)>0)) 
							{
								for (int k = 1; k<=currentLabel; k++)
									if (C[k] == C[labels(m,n)])
										C[k] = C[temp_label];
							}
						}

					
				}
		//printf("debug, %d\n",currentLabel);
		int labelSize[5000] = { 0 };
		for (size_t i = 1; i < 319; ++i)
			for (size_t j = 1; j < 239; ++j)
			{
				if ( labels(i,j) >0)
				{
					labels(i,j) = C[labels(i,j)];
					labelSize[labels(i,j)]++;
				}
			}

		int maxLabel = 1;
		int maxLabelSize = 1;
		for (size_t i = 0; i < currentLabel; ++i)
			if (labelSize[i]>maxLabelSize)
			{
				maxLabel = i;
				maxLabelSize = labelSize[i];
			}

		// output
		cloud_skin.clear();
		cloud_other.clear();
		PointCloudPtr current_cloud(new PointCloud);
		for (size_t i = 0; i < 320; ++i)
			for (size_t j = 0; j < 240; ++j)
			{
				if ((labels(i,j) == maxLabel) && (maxLabelSize>200))
				{
					cloud_skin.points.push_back(input_->at(i,j));
					current_cloud->points.push_back(input_->at(i,j));
				}
				else if ( !( (input_->at(i,j).r == 0) && (input_->at(i,j).g == 0) && (input_->at(i,j).b == 0)) )
					cloud_other.points.push_back(input_->at(i,j));
			}

		// store the most recent frames
		if(current_cloud->points.size()>300)
		{
			if (stored_frames<5)
			{
				pre_frame[stored_frames].swap(current_cloud);
				stored_frames++;
			}
			else
			{
				for(int i=0;i<4;i++)
					pre_frame[i].swap(pre_frame[i+1]);
				pre_frame[4].swap(current_cloud);

				total_sample_number_w = 0;
				for(int i=0;i<64;i++)
					for(int j=0;j<64;j++)
						skin_P_c_s_w[i][j] = 0;

				for(int i=0;i<5;i++)
					for (size_t j = 0; j < pre_frame[i]->points.size (); ++j)
					{
						int U, V;
						rgb2uv(pre_frame[i]->points[j].r, pre_frame[i]->points[j].g, pre_frame[i]->points[j].b, U, V);
						skin_P_c_s_w[U][V] ++;
						total_sample_number_w++;
					}

				skin_P_s_w = 0;
				for(int i=0; i < 64;i++)
					for(int j=0; j < 64;j++)
					{
						if (skin_P_c_s_w[i][j] > total_sample_number_w/25)
							skin_P_s_w ++;
					}
			}
		}
	}
}
#define PCL_INSTANTIATE_skin_tracking(T) template class PCL_EXPORTS pcl::skin_tracking<T>;


#endif  //#ifndef PCL_IMPL_SKINTRACKING_H_

