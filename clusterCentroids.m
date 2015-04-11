function [void] = clusterCentroids()
pylons_t UWClass_LIDAR_INT::cluster_centroids(float *scan_angles, float *scan_ranges, int num_scans, float min_range, float max_range, float delta, bool ramp_detect)
{
	float sum_ranges=0, sum_thetas=0, avg_range=0, avg_theta=0; int num_pts=0;	//temp vars
	int find_centroid = 0;	//start by not searching for centroid
	pylons_t cluster_peaks;	//pylon data

	float ramp_endx = 0;
	float ramp_endy = 0;
	float ramp_startx = 0;
	float ramp_starty = 0;
	
	cluster_peaks.total = -1;
	if (num_scans > 3)		//must have at least 3 scans for clustering to take place...
	{
		cluster_peaks.total = 0;
		for (int i=1; i<num_scans; i++)
		{
			float dist = fabs(scan_ranges[i] - scan_ranges[i-1]);		//jump distance
		
			if (dist >= delta)					//found the beginning of a peak
			{
				if (find_centroid == 1)			//was finding average of previous cluster
				{
					//find centroid in terms of range/theta, then convert to XY
					avg_theta = (float)(sum_thetas/(float)num_pts);
					avg_range = (float)(sum_ranges/(float)num_pts);
					cluster_peaks.X[cluster_peaks.total] = avg_range*cos(avg_theta);
					cluster_peaks.Y[cluster_peaks.total] = avg_range*sin(avg_theta);
					cluster_peaks.total++;

					// ramp detection (look for very very large "pylons"!)
					if(ramp_detect) //if we want to find ramp
					{
					
						ramp_endx = scan_ranges[i-1]*cos(scan_angles[i-1]);
						ramp_endy = scan_ranges[i-1]*sin(scan_angles[i-1]);					
				
					}
					
					float ramp_vectorx = ramp_endx - ramp_startx;
					float ramp_vectory = ramp_endy - ramp_starty;
					float r_l = sqrt(ramp_vectorx*ramp_vectorx + ramp_vectory*ramp_vectory); 

					ROS_INFO("r_l-> %f at -> %d", r_l,cluster_peaks.total);
			
					if(r_l>RAMP_THRESH) // most likley a ramp, remove this peak and pray that we dont hit anything :)
					{
						ramp_present = true;						
						cluster_peaks.total--;
						ramp_dist = avg_range;
												
					}
				}

				if (scan_ranges[i] >= min_range && scan_ranges[i] <= max_range)	//start of peak is within range limit
				{
					//initialize centroid finding
					sum_thetas = scan_angles[i];
					sum_ranges = scan_ranges[i];
					num_pts = 1;
					find_centroid = 1;				//start finding centroid
				
					//start ramp detect
					if(ramp_detect)
					{
			
						ramp_startx = scan_ranges[i]*cos(scan_angles[i]);
						ramp_starty = scan_ranges[i]*sin(scan_angles[i]);				
						
					}
				}
				else								//out of range limit
				{
					find_centroid = 0;				//ignore this peak
				}
			}
			else
			{
				if (find_centroid == 1)				//finding centroid while running along cluster
				{
					sum_thetas = sum_thetas + scan_angles[i];
					sum_ranges = sum_ranges + scan_ranges[i];
					num_pts++;
				}
			}	
		}
	}

	return cluster_peaks;
}