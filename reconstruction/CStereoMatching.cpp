#include "CStereoMatching.h"
#include <math.h>
#define DifferOfDisparity(x,y)	abs(x-y) > 1
#define MAX_DISPARITY 2
void CStereoMatching::Init(CManageData * data, CCloudOptimization * CloudOptimization, int radii, double ws, int disparity_offset)
{
	m_data = data;
	m_CloudOptimization = CloudOptimization;
	MatchBlockRadius = radii;
	m_ws = ws;
	m_offset = disparity_offset;
	Verbose = 1;
}

void CStereoMatching::MatchAllLayer()
{
	for (int CamPair=0; CamPair<m_data->m_CampairNum; CamPair++)
	{
		printf("processing pair %d: cam %d and cam %d...\n", CamPair, m_data->cam[CamPair][0].camID, m_data->cam[CamPair][1].camID);
		Rectify(CamPair, Q);
		ConstructPyrm(CamPair);
		cv::Mat disparity[2];
		for (int i = 0; i<m_data->m_PyrmNum; i++)
		{
 			MatchOneLayer(disparity, i);
		}
		m_data->cam[CamPair][0].bound = margin[0];
		m_data->cam[CamPair][1].bound = margin[1];
		DisparityToCloud<double>(disparity[0], m_data->maskPyrm[m_data->m_PyrmNum-1][0], Q, m_data->m_PyrmNum-1, true, CamPair);
#ifdef IS_PCL
		m_CloudOptimization->filter(CamPair);
#endif
	}
}

void CStereoMatching::MatchOneLayer(cv::Mat disparity[], int Pyrm_depth)
{
	if (Verbose>=1)
		printf("\tprocessing layer %d...\n",Pyrm_depth);
	clock_t start = clock();
	cv::Mat image[2], mask[2];
	cv::Mat image_inv[2], mask_inv[2];
	image[0] = m_data->imagePyrm[Pyrm_depth][0];
	image[1] = m_data->imagePyrm[Pyrm_depth][1];
	mask[0]  = m_data->maskPyrm[Pyrm_depth][0];
	mask[1]  = m_data->maskPyrm[Pyrm_depth][1];
	image_inv[0] = image[1];
	image_inv[1] = image[0];
	mask_inv[0] = mask[1];
	mask_inv[1] = mask[0];
	FindMargin(margin[0], m_data->maskPyrm[Pyrm_depth][0]);
	FindMargin(margin[1], m_data->maskPyrm[Pyrm_depth][1]);
	if (Pyrm_depth == 0)
	{
		LowestLevelInitialMatch(image, mask, disparity[0], true);
		LowestLevelInitialMatch(image_inv, mask_inv, disparity[1], false);
	}
	else
	{
		HighLevelInitialMatch(image, mask, disparity[0], Pyrm_depth, true);
		HighLevelInitialMatch(image_inv, mask_inv, disparity[1], Pyrm_depth, false);
	}
// 	DisparityToCloud<short>(disparity[0], mask[0], Q, 0, true, 10);
// 	m_data->SaveMat(disparity[0], "disparity0.dat");
// 	m_data->SaveMat(disparity[1], "disparity01.dat");
	SmoothConstraint(disparity[0], true);
	SmoothConstraint(disparity[1], false);
//	DisparityToCloud<short>(disparity[0], mask[0], Q, 0, true, 11);
	//m_data->SaveMat(disparity[0], "disparity1.dat");
	//m_data->SaveMat(disparity[1], "disparity11.dat");
	OrderConstraint(disparity[0], true);
	OrderConstraint(disparity[1], false);
	//m_data->SaveMat(disparity[0], "disparity2.dat");
	//m_data->SaveMat(disparity[1], "disparity21.dat");
	UniquenessContraint<short>(disparity);
//m_data->SaveMat(disparity[0], "disparity3.dat");
//m_data->SaveMat(disparity[1], "disparity31.dat");
// 	MedianFilter(disparity[0], mask[0], 1, true);
// 	MedianFilter(disparity[1], mask[1], 1, false);
	Rematch(image, mask, disparity[0], true);
	Rematch(image_inv, mask_inv, disparity[1], false);
// 	SmoothConstraint(disparity[0], true);
// 	SmoothConstraint(disparity[1], false);
// 	OrderConstraint(disparity[0], true);
// 	OrderConstraint(disparity[1], false);
	UniquenessContraint<short>(disparity);
//m_data->SaveMat(disparity[0], "disparity4.dat");
//m_data->SaveMat(disparity[1], "disparity41.dat");
	MedianFilter(disparity[0], mask[0], 1, true);
	MedianFilter(disparity[1], mask[1], 1, false);
	
//  m_data->SaveMat(disparity[0], "disparity50.dat");
//  m_data->SaveMat(disparity[1], "disparity51.dat");
// 	DisparityToCloud<short>(disparity[0], mask[0], Q, Pyrm_depth, true, 11+10*Pyrm_depth);
	int iteration = 30+Pyrm_depth*30;
//	clock_t part = clock();
	DisparityRefine(disparity[0], image, iteration, true);
	DisparityRefine(disparity[1], image_inv, iteration, false);
//	printf("\ttime: %.3f s\n", double(clock()-part)/1e3);
// 	disparity[0].convertTo(disparity[0],CV_64FC1);
// 	disparity[1].convertTo(disparity[1],CV_64FC1);
//	m_data->SaveMat(disparity[0], "disparity6.dat");
//	m_data->SaveMat(disparity[1], "disparity61.dat");
// 	char filename[512];
// 	sprintf(filename, "disparity%d_0.dat", Pyrm_depth);
// 	m_data->SaveMat(disparity[0], filename);
// 	sprintf(filename, "disparity%d_1.dat", Pyrm_depth);
// 	m_data->SaveMat(disparity[1], filename);
	UniquenessContraint<double>(disparity);
	
// 	m_data->SaveMat(disparity[0], "disparity70.dat");
	printf("\ttime: %.3f s\n", double(clock()-start)/1e3);
}

// input: cam_ID[0] for local camera ID
//		  cam_ID[1] for reference camera ID
void CStereoMatching::Rectify(int CamPair, cv::Mat &Q)
{
	printf("\trectifying...\n");
	cv::Size largestSize = m_data->m_LowestLevelSize*(1<<(m_data->m_PyrmNum-1));
	vector<camera> & current_cam = m_data->cam[CamPair];
	cv::Mat R,T;
	cv::Mat R_new[2];
	cv::Rect validRoi[2];
	R = current_cam[1].MatExtrinsics.colRange(0,3) * current_cam[0].MatExtrinsics.colRange(0,3).t();
	T = -R*current_cam[0].MatExtrinsics.col(3) + current_cam[1].MatExtrinsics.col(3);
	cv::Mat distCoeffs = cv::Mat::zeros(4,1,CV_64FC1);
	cv::stereoRectify(	current_cam[0].MatIntrinsics, distCoeffs,
		current_cam[1].MatIntrinsics, distCoeffs,
		m_data->m_OriginSize, R, T, R_new[0], R_new[1], current_cam[0].P, current_cam[1].P, Q,
		0, -1, m_data->m_OriginSize, &validRoi[0], &validRoi[1]);
	R_final = current_cam[0].MatExtrinsics.colRange(0,3).t() * R_new[0].t();
	T_final = -current_cam[0].MatExtrinsics.colRange(0,3).t() * current_cam[0].MatExtrinsics.col(3);
	cv::Mat Extrinsic_final = cv::Mat::zeros(4,4,CV_64FC1);
	Extrinsic_final.at<double>(3,3) = 1;
	Extrinsic_final(cv::Range(0,3), cv::Range(0,3)) = R_final.t();
	Extrinsic_final(cv::Range(0,3), cv::Range(3,4)) = -R_final.t()*T_final;
	Q.at<double>(3,2) = -Q.at<double>(3,2);
	cv::Mat rmap[2], img;
	double scale = double(m_data->m_LowestLevelSize.width) / m_data->m_OriginSize.width * (1<<(m_data->m_PyrmNum-1)); 
	temp_P02 = current_cam[1].P.at<double>(0,2);
	for (int j=0; j<2; j++)
	{
		current_cam[j].P.rowRange(0,2) *= scale;
		initUndistortRectifyMap(current_cam[j].MatIntrinsics, distCoeffs, R_new[j], current_cam[j].P, largestSize, CV_16SC2, rmap[0], rmap[1]);
		current_cam[j].P = current_cam[j].P*Extrinsic_final;
		img = cv::imread(current_cam[j].image_name);
		if (img.empty() == true)
		{
			printf("read image %s error\n", current_cam[j].image_name.c_str());
			return ;
		}
		
//		cv::medianBlur(img,img,5);
		cv::remap(img, current_cam[j].image, rmap[0], rmap[1], CV_INTER_LINEAR);
		//cout<<rmap[0].rows<<endl;
// 		cv::imshow("img", current_cam[j].image);
// 		cv::waitKey(0);
//		cv::medianBlur(current_cam[j].image,current_cam[j].image,5);
		img = cv::imread(current_cam[j].mask_name, CV_LOAD_IMAGE_GRAYSCALE);
		cv::remap(img, current_cam[j].mask, rmap[0], rmap[1], CV_INTER_LINEAR);
		cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 3*(1<<(m_data->m_PyrmNum-1)), 3*(1<<(m_data->m_PyrmNum-1) )));
		cv::erode(current_cam[j].mask, current_cam[j].mask, element);
		if (m_data->isoutput)
		{
			char filename[MAX_PATH];
			sprintf(filename, "%d_%d.jpg", CamPair, current_cam[j].camID);
			cv::imwrite(filename, current_cam[j].image);
	// 		sprintf(filename, "%d_mask.jpg", current_cam[j].camID);
	// 		cv::imwrite(filename, current_cam[j].mask);
		}
	}
}

void CStereoMatching::LowestLevelInitialMatch(cv::Mat image[], cv::Mat mask[], cv::Mat &disparity, bool IsZeroOne)
{
	if (Verbose>=2)
		printf("\t\tlowest level initial match starts...\n");
	disparity = cv::Mat(m_data->m_LowestLevelSize, CV_16SC1, cv::Scalar(NOMATCH));
	int YL, YR, XL, XR, XL1, XR1;
	YL = margin[!IsZeroOne].YL;
	YR = margin[!IsZeroOne].YR;
	XL = margin[!IsZeroOne].XL;
	XR = margin[!IsZeroOne].XR;
	XL1 = margin[IsZeroOne].XL;
	XR1 = margin[IsZeroOne].XR;
	int window_size = 2*MatchBlockRadius+1;
	int vec_size = square_(window_size)*3;
#pragma omp parallel for
	for (int y=YL; y<=YR; y++)
	{
		uchar *p = mask[0].ptr<uchar>(y);
		uchar *q = mask[1].ptr<uchar>(y);
		short *s = disparity.ptr<short>(y);
		uchar ** window_ptrL = new uchar *[window_size];
		uchar ** window_ptrR = new uchar *[window_size];
		for (int i=-MatchBlockRadius; i<=MatchBlockRadius; i++)
		{
			window_ptrL[i+MatchBlockRadius] = image[0].ptr<uchar>(y+i);
			window_ptrR[i+MatchBlockRadius] = image[1].ptr<uchar>(y+i);
		}
		for (int x=XL; x<=XR; x++)
		{
			// mask
			if (p[x] != 255)// || window_ptrL[MatchBlockRadius][XL*3] > 200 || window_ptrL[MatchBlockRadius][XL*3+1] > 200 || window_ptrL[MatchBlockRadius][XL*3] > 200)
				continue;
			arma::vec vecL(vec_size), vecR(vec_size);
			double normL = m_data->WindowToVec(window_ptrL, x-MatchBlockRadius, window_size, vecL);
			vecL /= normL;
			short temp_i = -1;
			double CurrentMaxValue = -1;
			for (int iMatch=XL1; iMatch<=XR1; iMatch++)
			{
				if (q[iMatch] != 255)
					continue;
				double normR = m_data->WindowToVec(window_ptrR, iMatch-MatchBlockRadius, window_size, vecR);
				double CurrentValue = arma::dot(vecL, vecR)/normR;
				if (CurrentValue > CurrentMaxValue)
				{
					temp_i = iMatch;
					CurrentMaxValue = CurrentValue;
				}
			}
			if (temp_i != -1)
			{
				s[x] = short(temp_i - x);
			}
		}
		delete [] window_ptrL;
		delete [] window_ptrR;
	}
}

// input: src_disparity<double>
// output: dst_disparity<short>
void CStereoMatching::HighLevelInitialMatch(cv::Mat image[], cv::Mat mask[], cv::Mat &disparity, int Pyrm_depth, bool IsZeroOne)
{
	if (Verbose>=2)
		printf("\t\thigh level initial match starts...\n");
	cv::Mat dst_disparity(image[0].size(), CV_16S, cv::Scalar(NOMATCH));
	int YL, YR, XL, XR, XL1, XR1, YR1;
	YL = margin[!IsZeroOne].YL;
	YR = margin[!IsZeroOne].YR;
	XL = margin[!IsZeroOne].XL;
	XR = margin[!IsZeroOne].XR;
	XL1 = margin[IsZeroOne].XL;
	XR1 = margin[IsZeroOne].XR;
	YR1 = margin[IsZeroOne].YR;
	int window_size = 2*MatchBlockRadius+1;
	int vec_size = square_(window_size)*3;
#pragma omp parallel for
	for (int y=YL; y<=YR; y++)
	{
		uchar *p = mask[0].ptr<uchar>(y);
		uchar *q = mask[1].ptr<uchar>(y);
		uchar ** window_ptrL = new uchar *[window_size];
		uchar ** window_ptrR = new uchar *[window_size];
		for (int i=-MatchBlockRadius; i<=MatchBlockRadius; i++)
		{
			window_ptrL[i+MatchBlockRadius] = image[0].ptr<uchar>(y+i);
			window_ptrR[i+MatchBlockRadius] = image[1].ptr<uchar>(y+i);
		}
		short *d = dst_disparity.ptr<short>(y);
		double *s = disparity.ptr<double>(int((y+1)/2.0));//@@没看懂MIN(int((y+1)/2.0), YR1)
		int boundary_L = XL1;
		int boundary_R = XR1;
		for (int x=XL; x<=XR; x++)
		{
			// mask
			if (p[x] != 255)// || window_ptrL[MatchBlockRadius][XL*3] > 200 || window_ptrL[MatchBlockRadius][XL*3+1] > 200 || window_ptrL[MatchBlockRadius][XL*3] > 200)
				continue;
			int temp2 = int((x+1)/2.0);//@@没看懂MIN(int((x+1)/2.0), XR1>>1)
			arma::vec vecL(vec_size), vecR(vec_size);
			double normL = m_data->WindowToVec(window_ptrL, x-MatchBlockRadius, window_size, vecL);
			vecL /= normL;
			ushort temp_i;
			double CurrentMaxValue = -1;
			if (s[temp2] == NOMATCH)
			{
				for (int i=temp2+1; i<=XR>>1 ; i++)
				{
					if (s[i] != NOMATCH)
					{
						boundary_R = MIN(i + int(s[i]*2) + m_offset + 1, XR1);
						break;
					}
				}
			}
			else
			{
				boundary_L = MAX(x + int(s[temp2]*2+0.5) - m_offset, XL1);
				boundary_R = MIN(x + int(s[temp2]*2+0.5) + m_offset, XR1);
			}
			for (int iMatch=boundary_L; iMatch<=boundary_R; iMatch++)
			{
				if (q[iMatch] != 255)
					continue;
				double normR = m_data->WindowToVec(window_ptrR, iMatch-MatchBlockRadius, window_size, vecR);
				double CurrentValue = arma::dot(vecL, vecR)/normR;
				if (CurrentValue > CurrentMaxValue)
				{
					CurrentMaxValue = CurrentValue;
					temp_i = iMatch;
				}
			}
			if (CurrentMaxValue > -1)
				d[x] = short(temp_i - x);
		}
		delete [] window_ptrL;
		delete [] window_ptrR;
	}
	dst_disparity.copyTo(disparity);
}

void CStereoMatching::OrderConstraint(cv::Mat disparity, bool IsZeroOne)
{
	if (Verbose>=2)
		printf("\t\tadding ordering constraint...\n");
	int YL, YR, XL, XR;
	YL = margin[!IsZeroOne].YL;
	YR = margin[!IsZeroOne].YR;
	XL = margin[!IsZeroOne].XL;
	XR = margin[!IsZeroOne].XR;
	int max_L = XR-XL+1;
#pragma omp parallel for
	for (int y=YL; y<=YR; y++)
	{
		short *disparity_line = new short[max_L];
		short *disparity_line_index = new short[max_L];
		short *p = disparity.ptr<short>(y);
		// choose those valid points
		int valid_count = 0;
		for (int x=XL; x<=XR; x++)
		{
			if (p[x] == NOMATCH)
				continue;
			disparity_line[valid_count] = p[x] + x;
			disparity_line_index[valid_count] = x;
			valid_count++;
		}
		// construct cross matrix
		arma::Mat<short> A(valid_count, valid_count);
		A.zeros();
		int ones_count = 0;
		for (int i=0; i<valid_count; i++)
		{
			for (int j=0; j<i; j++)
			{
				if (disparity_line[j] > disparity_line[i])
				{
					A(i,j) = 1;
					ones_count++;
				}
			}
		}
		A = A + arma::trans(A);
		// delete the points with the biggest crosses
		arma::Col<short> cross_count = arma::sum(A, 1);
		while (ones_count)
		{
			arma::uword max_cross_index;
			int max_cross = cross_count.max(max_cross_index);
			cross_count -= A.col(max_cross_index);
			cross_count(max_cross_index) = 0;
			A.row(max_cross_index).zeros();
			A.col(max_cross_index).zeros();
			ones_count -= max_cross;
			p[disparity_line_index[max_cross_index]] = NOMATCH;
		}
		delete [] disparity_line;
		delete [] disparity_line_index;
	}
}

void CStereoMatching::SmoothConstraint(cv::Mat disparity, bool IsZeroOne)
{
	if (Verbose>=2)
		printf("\t\tadding smoothness constraint...\n");
	int YL, YR, XL, XR;
	YL = margin[!IsZeroOne].YL;
	YR = margin[!IsZeroOne].YR;
	XL = margin[!IsZeroOne].XL;
	XR = margin[!IsZeroOne].XR;
	cv::Mat TempMat(disparity.rows, disparity.cols, CV_8UC2, cv::Scalar(0,0));	//first:num of exist total;second num of effective total(diff>=1)
	for (int y=YL; y<=YR; y++)
	{
		short *pup = disparity.ptr<short>(y);
		short *pdown = disparity.ptr<short>(y+1);
		uchar *qup = TempMat.ptr<uchar>(y);
		uchar *qdown = TempMat.ptr<uchar>(y+1);
		for (int x=XL; x<=XR ; x++)
		{
			if (pup[x] == NOMATCH)
				continue;
			int double_x = x<<1;
			if (pup[x+1] != NOMATCH)	//east
			{
				qup[double_x]++;
				qup[double_x+2]++;
				if (DifferOfDisparity(pup[x], pup[x+1]))
				{
					qup[double_x+1]++;
					qup[double_x+3]++;
				}
			}			
			if (pdown[x-1] != NOMATCH)//southwest
			{
				qup[double_x]++;
				qdown[double_x-2]++;
				if (DifferOfDisparity(pup[x], pdown[x-1]))
				{
					qup[double_x+1]++;
					qdown[double_x-1]++;
				}
			}			
			if (pdown[x] != NOMATCH)//south
			{
				qup[double_x]++;
				qdown[double_x]++;
				if (DifferOfDisparity(pup[x], pdown[x]))
				{
					qup[double_x+1]++;
					qdown[double_x+1]++;
				}
			}			
			if (pdown[x+1] != NOMATCH)//southeast
			{
				qup[x]++;
				qdown[x+2]++;
				if (DifferOfDisparity(pup[x], pdown[x+1]))
				{
					qup[double_x+1]++;
					qdown[double_x+3]++;
				}
			}
		}
	}	
	//take out useless pixel
#pragma omp parallel for
	for (int y = YL; y<=YR; y++)
	{
		uchar *pcheck = TempMat.ptr<uchar>(y) + (XL<<1);
		short *pDis = disparity.ptr<short>(y);
		for (int x = XL; x<=XR ; x++)
		{
			//not exist or 8 pixels around all NOMATCH
			//more than half of pixels around don't satisfy 
			if( (pcheck[0] == 0) || (pcheck[1]<<1 > pcheck[0]))
				pDis[x] = NOMATCH;
			pcheck += 2;
		}
	}
}

template<class T>
void CStereoMatching::UniquenessContraint(cv::Mat disparity[])
{
	if (Verbose>=2)
		printf("\t\tadding uniqueness constraint...\n");
	cv::Mat disparity_inv[2];
	UniquenessContraint_<T>(disparity, true);
	disparity_inv[0] = disparity[1];
	disparity_inv[1] = disparity[0];
	UniquenessContraint_<T>(disparity_inv, false);
	UniquenessContraint_<T>(disparity, true);
}

template<class T>
void CStereoMatching::UniquenessContraint_(cv::Mat disparity[], bool IsZeroOne)
{
	int YL, YR, XL, XR, XL1, XR1;
	YL = margin[!IsZeroOne].YL;
	YR = margin[!IsZeroOne].YR;
	XL = margin[!IsZeroOne].XL;
	XR = margin[!IsZeroOne].XR;
	XL1 = margin[IsZeroOne].XL;
	XR1 = margin[IsZeroOne].XR;
#pragma omp parallel for	
	for (int y=YL; y<=YR; y++)
	{
		T *p = disparity[0].ptr<T>(y);
		T *q = disparity[1].ptr<T>(y);
		for (int x=XL; x<=XR; x++)
		{
			if (p[x] == NOMATCH)
				continue;
			int boundary_L = MAX(int(p[x]+0.5) + x - 1, XL1);
			int boundary_R = MIN(boundary_L + 2, XR1);
			int iMatch;
			for (iMatch=boundary_L; iMatch<=boundary_R; iMatch++)
			{
				if (abs(q[iMatch] + p[x]) < 2)
					break;
			}
			if (iMatch > boundary_R)
			{	
				if ( abs(q[boundary_L+1] + p[x-1])>=2 && abs(q[boundary_L+1] + p[x+1])>=2 )
					p[x] = NOMATCH;
			}
		}
	}
}

void CStereoMatching::Rematch(cv::Mat image[], cv::Mat mask[], cv::Mat disparity, bool IsZeroOne)
{
	if (Verbose>=2)
		printf("\t\trematching...\n");
	int YL, YR, XL, XR, XL1, XR1;
	YL = margin[!IsZeroOne].YL;
	YR = margin[!IsZeroOne].YR;
	XL = margin[!IsZeroOne].XL;
	XR = margin[!IsZeroOne].XR;
	XL1 = margin[IsZeroOne].XL;
	XR1 = margin[IsZeroOne].XR;
	int window_size = 2*MatchBlockRadius+1;
	int vec_size = square_(window_size)*3;

	cv::Mat BL, BR;
	SetBoundary_smooth<short>(disparity, mask[0], BL, BR, IsZeroOne);
//	m_data->SaveMat(BL,"bl.dat");
#pragma omp parallel for
	for (int y=YL; y<=YR; y++)
	{
		uchar *p = mask[0].ptr<uchar>(y);
		uchar *q = mask[1].ptr<uchar>(y);
		short *s = disparity.ptr<short>(y);
		uchar ** window_ptrL = new uchar *[window_size];
		uchar ** window_ptrR = new uchar *[window_size];
		short *bl_src = BL.ptr<short>(y);
		short *br_src = BR.ptr<short>(y);
		for (int i=-MatchBlockRadius; i<=MatchBlockRadius; i++)
		{
			window_ptrL[i+MatchBlockRadius] = image[0].ptr<uchar>(y+i);
			window_ptrR[i+MatchBlockRadius] = image[1].ptr<uchar>(y+i);
		}
		for (int x=XL; x<=XR ; x++)
		{
			if (p[x] != 255)
				continue;
			arma::vec vecL(vec_size), vecR(vec_size);
			double normL = m_data->WindowToVec(window_ptrL, x-MatchBlockRadius, window_size, vecL);
			vecL /= normL;
			if (s[x] == NOMATCH)
			{
				// rematch
				short temp_i = -1;
				double CurrentMaxValue = -1;
				int L=(int)bl_src[x];
				int R=(int)br_src[x];
// 				if (L>R)
// 				{
// 					int temp = L;
// 					L = R;
// 					R = temp;
// 				}
				for (int iMatch=L; iMatch<=R; iMatch++)
				{
					if (q[iMatch] != 255)
						continue;
					double normR = m_data->WindowToVec(window_ptrR, iMatch-MatchBlockRadius, window_size, vecR);
					double CurrentValue = arma::dot(vecL, vecR)/normR;
					if (CurrentValue > CurrentMaxValue)
					{
						temp_i = iMatch;
						CurrentMaxValue = CurrentValue;
					}
				}
				if (temp_i != -1)
					s[x] = short(temp_i - x);
			}
		}
		delete [] window_ptrL;
		delete [] window_ptrR;
	}
}

void CStereoMatching::DisparityRefine(cv::Mat &disparity, cv::Mat image[], int iteration, bool IsZeroOne)
{
	if (Verbose>=2)
		printf("\t\tdisparity refining...\n");
	int YL, YR, XL, XR, XL1, XR1;
	YL = margin[!IsZeroOne].YL;
	YR = margin[!IsZeroOne].YR;
	XL = margin[!IsZeroOne].XL;
	XR = margin[!IsZeroOne].XR;
	XL1 = margin[IsZeroOne].XL;
	XR1 = margin[IsZeroOne].XR;

	cv::Mat disparity_out;
	disparity.convertTo(disparity_out, CV_64FC1);
	cv::Mat Current_Disparity;
	disparity_out.copyTo(Current_Disparity);
	disparity.release();
	//iterate and calculate d'
	for (int iter=0; iter<iteration; iter++)
	{
//		sprintf(filename, "disparity%d.dat", iter);
//		m_data->SaveMat(disparity_out, filename);
#pragma omp parallel for
		for (int y=YL+1; y<=YR-1; y++)
		{
			double *pdis0 = disparity_out.ptr<double>(y-1);	//one dimension,point up and down
			double *pdis1 = disparity_out.ptr<double>(y); 
			double *pdis2 = disparity_out.ptr<double>(y+1);
			double *pCurrent_disp = Current_Disparity.ptr<double>(y);
			arma::vec vecL(27), vecR(27);
			double normL, normR;
			double pdp,pwp;
			uchar * window_ptrL[3], * window_ptrR[3];
			double xi[3];
			for (int i=-1; i<=1; i++)
			{
				window_ptrL[i+1] = image[0].ptr<uchar>(y+i);
				window_ptrR[i+1] = image[1].ptr<uchar>(y+i);
			}
			for (int x=XL+1; x<=XR-1; x++)
			{
				if (pdis1[x] == NOMATCH)
					continue;
				double dCenter = pdis1[x];
				double dEast = pdis1[x+1];
				double dWest = pdis1[x-1];
				double dNorth = pdis0[x];
				double dSouth = pdis2[x];
				char mode = (dEast != NOMATCH && dWest != NOMATCH) + char(dSouth != NOMATCH && dNorth != NOMATCH) * 2 ;
				assert(dCenter == dCenter);
				if (mode != 0)
				{
					normL = m_data->WindowToVec(window_ptrL, x-1, 3, vecL);
					int iMatch = int(dCenter-1.5)+x;
					for (int i=0; i<3; i++)
					{
						normR = m_data->WindowToVec(window_ptrR, iMatch+i, 3, vecR);
						xi[i] = (1-arma::dot(vecL,vecR)/(normL*normR))/2;
					}
					int index = xi[0] >= xi[1];
					if (xi[index] > xi[2]) index = 2;
					switch(index)
					{
					case 0:
						pwp = xi[1] - xi[0];
						pdp = dCenter - 0.5;
						break;
					case 1:
						pwp = 0.5*(xi[0]+xi[2]) - xi[1];
						pdp = dCenter + 0.5*(xi[0]-xi[2])/(xi[0]+xi[2]-2*xi[1]);
						if (pwp == 0)
							pdp = 0;
						break;
					case 2:
						pwp = xi[1] - xi[2];
						pdp = dCenter + 0.5;
						break;
					default:;
					}
				}
				switch(mode)
				{
				case 0:
					pCurrent_disp[x] = dCenter;
					break;
				case 1:
					pCurrent_disp[x] = (pdp*pwp + m_ws*(dEast + dWest)/2) / (pwp + m_ws);
					break;
				case 2:
					pCurrent_disp[x] = (pdp*pwp + m_ws*(dNorth + dSouth)/2) / (pwp + m_ws);
					break;
				case 3:
					double wx, wy, ds;
					wx = exp( -square_(abs(dEast - dCenter) - abs(dWest - dCenter)) );
					wy = exp( -square_(abs(dSouth - dCenter) - abs(dNorth - dCenter)) );
					if (wx+wy == 0)
						ds = (dEast + dWest + dSouth + dNorth)/4;
					else
						ds = ( wx*(dEast + dWest) + wy*(dNorth + dSouth) ) / (2*(wx+wy));
					pCurrent_disp[x] = (pdp*pwp + m_ws*ds) / (pwp + m_ws);
				}
			}
		}
		disparity = disparity_out;
		disparity_out = Current_Disparity;
		Current_Disparity = disparity;
	}
	disparity = disparity_out;
}

template<class T>
void CStereoMatching::DisparityToCloud(cv::Mat disparity, cv::Mat mask_org, cv::Mat Q, int Pyrm_depth, bool IsZeroOne, int idx)
{	
	if (Verbose>=1)
		printf("\tconverting disparity to cloud %d...\n", idx);
	int YL, YR, XL, XR;
	YL = margin[!IsZeroOne].YL;
	YR = margin[!IsZeroOne].YR;
	XL = margin[!IsZeroOne].XL;
	XR = margin[!IsZeroOne].XR;
	double scale = double(m_data->m_LowestLevelSize.width) / m_data->m_OriginSize.width * (1<<Pyrm_depth); 
	cv::Mat image_scaled = m_data->imagePyrm[Pyrm_depth][!IsZeroOne];
	
	double q[4][4];
	cv::Mat _Q(4, 4, CV_64F, q);
	Q.convertTo(_Q, CV_64F);
	_Q.col(3) *= scale;
// 	if(!IsZeroOne){
// 		idx = (idx-1)>>1;
// 		_Q.at<double>(0,3) = -temp_P02*scale;
// 		_Q.at<double>(3,2) = -_Q.at<double>(3,2);
// 		_Q.at<double>(3,3) = -_Q.at<double>(3,3);
// 		cout<<_Q<<endl;
// 	}
	double qz = q[2][3], qw = q[3][3];
	double _Fout[3][1];
	cv::Mat Fout(3,1,CV_64FC1, _Fout);
	cv::Mat mask = mask_org.clone();
	int erode_size = ceil(0.02 * mask_org.rows);
 	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( erode_size, erode_size ));
 	cv::erode(mask, mask, element);
	FILE* fp;
	if (m_data->isoutput){
		char filename[20];
		int point_count = 0;
		for( int y = YL; y <= YR; y++ )
		{
			T *sptr = disparity.ptr<T>(y);
			uchar *p = mask.ptr<uchar>(y);
			for( int x = XL; x <= XR; x++ )
			{
				if (p[x] != 255)				
					continue;
				if (sptr[x] == NOMATCH)				
					continue;
				point_count++;
			}
		}
		sprintf(filename, "cloud%d.ply", idx);
		fp = fopen(filename, "wb");
		fprintf(fp, "ply\n");
		fprintf(fp, "format binary_little_endian 1.0\n");
		fprintf(fp, "element vertex %d\n", point_count);
		fprintf(fp, "property float x\nproperty float y\nproperty float z\nproperty uchar blue\nproperty uchar green\nproperty uchar red\n");//
		fprintf(fp, "end_header\n");
	}

	for( int y = YL; y <= YR; y++ )
	{
		T *sptr = disparity.ptr<T>(y);
		uchar *s = image_scaled.ptr<uchar>(y)+(XL-1)*3;
		double qy = y + q[1][3];
		uchar *p = mask.ptr<uchar>(y);
		for( int x = XL; x <= XR; x++ )
		{
			s += 3;
			if (p[x] != 255)
				continue;
			if (sptr[x] == NOMATCH)				
				continue;
			double iW = 1./(qw + q[3][2]*sptr[x]);
			_Fout[0][0] = (q[0][3] + double(x))*iW;
			_Fout[1][0] = qy*iW;
			_Fout[2][0] = qz*iW;
			cv::Mat point = R_final*Fout+T_final;
#ifdef IS_PCL
			m_CloudOptimization->InsertPoint(point);
#endif
			if (m_data->isoutput){
				point.convertTo(point, CV_32FC1);
				fwrite(point.data, sizeof(float), 3, fp);
				fwrite(s, 1, 3, fp);
			}
		}
	}
	if (m_data->isoutput) fclose(fp);
}

void CStereoMatching::MedianFilter(cv::Mat &disparity, cv::Mat mask, int iteration, bool IsZeroOne)
{
	if (Verbose>=2)
		printf("\t\tmedian filtering...\n");
	int YL, YR, XL, XR;
	YL = margin[!IsZeroOne].YL;
	YR = margin[!IsZeroOne].YR;
	XL = margin[!IsZeroOne].XL;
	XR = margin[!IsZeroOne].XR;
	cv::Mat disparity_temp(disparity.size(),CV_16SC1,cv::Scalar(NOMATCH));
	cv::Mat swap;
	for (int i=0; i<iteration; i++)
	{
#pragma omp parallel for
		for( int y = YL; y <= YR; y++ )
		{
			short ** window_ptr = new short *[3];
			uchar * mask_ptr = mask.ptr<uchar>(y);
			short * p = disparity_temp.ptr<short>(y);
			for (int i=-1; i<=1; i++)
			{
				window_ptr[i+1] = disparity.ptr<short>(y+i);
			}
			for( int x = XL; x <= XR; x++ )
			{
				if (mask_ptr[x] != 255)
					continue;
				arma::ivec u(9);
				int k = 0;
				for (int i = x-1; i < x+1; i++)
					for (int j = 0; j < 3; j++)
						if (window_ptr[j][i] != NOMATCH)
							u(k++) = window_ptr[j][i];			
				if (window_ptr[1][x] == NOMATCH)
				{
					if (k >= 4)
						p[x] = arma::median(u.rows(0,k-1));
					else p[x] = NOMATCH;
				}
				else
				{
					if (k <= 2)
						p[x] = NOMATCH;
					else p[x] = arma::median(u.rows(0,k-1));
				}
			}
			delete [] window_ptr;
		}
		swap = disparity_temp;
		disparity_temp = disparity;
		disparity = swap;
	}
}

template<class T>
void CStereoMatching::SetBoundary_smooth(cv::Mat disparity, cv::Mat mask, cv::Mat &boundary_L, cv::Mat &boundary_R, bool IsZeroOne)
{
	int YL, YR, XL, XR, XL1, XR1;
	YL = margin[!IsZeroOne].YL;
	YR = margin[!IsZeroOne].YR;
	XL = margin[!IsZeroOne].XL;
	XR = margin[!IsZeroOne].XR;
	XL1 = margin[IsZeroOne].XL;
	XR1 = margin[IsZeroOne].XR;
	if (YL >= YR || XL >= XR) {
		printf("YL(%d)>=YR(%d) || XL(%d)>=XR(%d)\n", YL, YR, XL, XR);
		exit(0);
	}
	//set boundary
	boundary_L = cv::Mat(disparity.size(), CV_16SC1, cv::Scalar(-10000));
	boundary_R = cv::Mat(disparity.size(), CV_16SC1, cv::Scalar(10000));
	T *src;
	short *bl_src, *br_src, *bl_dst1, *br_dst1, *bl_dst2, *br_dst2;
	uchar * mask_ptr;

	//up and down
	bl_dst2 = boundary_L.ptr<short>(YL);
	br_dst2 = boundary_R.ptr<short>(YL);

	for (int y=YL; y<=YR-1; y++)
	{
		src = disparity.ptr<T>(y);
		mask_ptr = mask.ptr<uchar>(y);
		bl_src = bl_dst2;
		bl_dst2 = boundary_L.ptr<short>(y+1);
		br_src = br_dst2;	
		br_dst2 = boundary_R.ptr<short>(y+1);
#pragma omp parallel for
		for (int x=XL; x<=XR ; x++)
		{
			if (mask_ptr[x]!=255)
				continue;
			T ref_value = src[x];
			if (ref_value==NOMATCH)
			{
				bl_dst2[x] = MAX(bl_src[x]-MAX_DISPARITY, bl_dst2[x]);
				br_dst2[x] = MIN(br_src[x]+MAX_DISPARITY, br_dst2[x]);
			}
			else
			{
				bl_src[x] = ref_value;
				br_src[x] = ref_value;
				bl_dst2[x] = MAX(ref_value-MAX_DISPARITY, bl_dst2[x]);
				br_dst2[x] = MIN(ref_value+MAX_DISPARITY, br_dst2[x]);
			}
		}
	}
	
	//reverse up and down
	bl_dst1 = boundary_L.ptr<short>(YR);
	br_dst1 = boundary_R.ptr<short>(YR);
	for (int y=YR; y>=YL+1; y--)
	{
		src = disparity.ptr<T>(y);
		mask_ptr = mask.ptr<uchar>(y);
		bl_src = bl_dst1;
		bl_dst1 = boundary_L.ptr<short>(y-1);
		br_src = br_dst1;	
		br_dst1 = boundary_R.ptr<short>(y-1);
#pragma omp parallel for
		for (int x=XL; x<=XR ; x++)
		{
			if (mask_ptr[x]!=255)
				continue;
			T ref_value = src[x];
			if (ref_value==NOMATCH)
			{
				bl_dst1[x] = MAX(bl_src[x]-MAX_DISPARITY, bl_dst1[x]);
				br_dst1[x] = MIN(br_src[x]+MAX_DISPARITY, br_dst1[x]);
			}
			else
			{
				bl_src[x] = ref_value;
				br_src[x] = ref_value;
				bl_dst1[x] = MAX(ref_value-MAX_DISPARITY, bl_dst1[x]);
				br_dst1[x] = MIN(ref_value+MAX_DISPARITY, br_dst1[x]);
			}
		}
	}
	//left and right
#pragma omp parallel for
	for (int y=YL; y<=YR; y++)
	{
		short * bl_src_ = boundary_L.ptr<short>(y);
		short * br_src_ = boundary_R.ptr<short>(y);
		uchar * mask_ptr_ = mask.ptr<uchar>(y);
		for (int x=XL; x<=XR-1 ; x++)
		{
			if (mask_ptr_[x]==255)
			{
				bl_src_[x+1] = MAX(bl_src_[x]-1, bl_src_[x+1]);
				br_src_[x+1] = MIN(br_src_[x]+MAX_DISPARITY, br_src_[x+1]);
			}
		}
		for (int x=XR; x>=XL+1 ; x--)
		{
			if (mask_ptr_[x]==255)
			{
				bl_src_[x] += x;
				br_src_[x] += x;
				if (bl_src_[x]<XL1)
					bl_src_[x]=XL1;
				if (br_src_[x]>XR1)
					br_src_[x]=XR1;
				bl_src_[x-1] = MAX(bl_src_[x]-x-MAX_DISPARITY, bl_src_[x-1]);
				br_src_[x-1] = MIN(br_src_[x]-x+1, br_src_[x-1]);
				
			}
		}
		if (mask_ptr_[XL]==255)
		{
			bl_src_[XL] += XL;
			br_src_[XL] += XL;
			if (bl_src_[XL]<XL1)
				bl_src_[XL]=XL1;
			if (br_src_[XL]>XR1)
				bl_src_[XL]=XR1;
		}
	}
}

template<class T>
void CStereoMatching::SetBoundary(cv::Mat disparity, cv::Mat mask, cv::Mat &boundary_L, cv::Mat &boundary_R, bool IsZeroOne)
{
	int YL, YR, XL, XR, XL1, XR1;
	YL = margin[!IsZeroOne].YL;
	YR = margin[!IsZeroOne].YR;
	XL = margin[!IsZeroOne].XL;
	XR = margin[!IsZeroOne].XR;
	XL1 = margin[IsZeroOne].XL;
	XR1 = margin[IsZeroOne].XR;
	if (YL >= YR || XL >= XR) 
	{
		printf("YL(%d)>=YR(%d) || XL(%d)>=XR(%d)\n", YL, YR, XL, XR);
		exit(0);
	}
	//set boundary
	boundary_L = cv::Mat(disparity.size(), CV_16SC1, cv::Scalar(XL1));
	boundary_R = cv::Mat(disparity.size(), CV_16SC1, cv::Scalar(XR1));

#pragma omp parallel for
	for (int y=YL; y<=YR; y++)
	{
		int bl=XL1, br=XR1;
		uchar * mask_ptr = mask.ptr<uchar>(y);
		T * p = disparity.ptr<T>(y);
		short * bl_src = boundary_L.ptr<short>(y);
		short * br_src = boundary_R.ptr<short>(y);
		for (int x=XL; x<=XR; x++)
		{
			if (mask_ptr[x] != 255)
				continue;
			if (p[x] != NOMATCH)
			{
				br = MIN(p[x]+x,XR1);
				break;
			}
		}
		for (int x=XL; x<=XR; x++)
		{
			if (mask_ptr[x] != 255)
				continue;
			if (p[x] == NOMATCH)
			{
				bl_src[x] = bl;
				br_src[x] = br;
			}
			else
			{
				bl = br;
				br = XR1;
				bl_src[x] = bl;
				br_src[x] = bl;
				for (int x1=x+1; x1<=XR; x1++)
				{
					if (mask_ptr[x1] != 255)
						continue;
					if (p[x1] != NOMATCH)
					{
						br = MIN(p[x1]+x1,XR1);
						break;
					}
				}
			}
		}
	}
}

void CStereoMatching::FindMargin(Boundary &m, cv::Mat mask)
{
	cv::Size newsize = mask.size();
	m.YL = newsize.height-1-MatchBlockRadius;
	m.YR = MatchBlockRadius;
	m.XL = newsize.width-1-MatchBlockRadius;
	m.XR = MatchBlockRadius;
	for (int y=MatchBlockRadius; y<newsize.height-MatchBlockRadius; y++)
	{
		uchar *p = mask.ptr<uchar>(y);
		bool flag = false;
		for (int x=MatchBlockRadius; x<newsize.width-MatchBlockRadius; x++)
		{
			if (p[x] != 255)
				continue;
			m.XL = MIN(m.XL, x);
			m.XR = MAX(m.XR, x);
			flag = true;
		}
		if (flag == true)
		{
			m.YL = MIN(m.YL, y);
			m.YR = MAX(m.YR, y);
		}
	}
	m.width = m.XR-m.XL+1;
	m.height = m.YR-m.YL+1;
}

void CStereoMatching::ConstructPyrm(int CamPair)
{
	
	for (int ID=0; ID<2; ID++)
	{
		m_data->cam[CamPair][ID].image.copyTo(m_data->imagePyrm[m_data->m_PyrmNum-1][ID]);
		m_data->cam[CamPair][ID].mask.copyTo(m_data->maskPyrm[m_data->m_PyrmNum-1][ID]);
		for (int i=m_data->m_PyrmNum-1; i>0; i--)
		{
			pyrDown(m_data->imagePyrm[i][ID], m_data->imagePyrm[i-1][ID]);
			pyrDown(m_data->maskPyrm[i][ID], m_data->maskPyrm[i-1][ID]);
		}
	}
}