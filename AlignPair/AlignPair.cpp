// AlignPair.cpp : Defines the entry point for the console application.

#include "SiftFeature.h"
#include "Homography.h"
#include <time.h>
#include <algorithm>
using namespace std;

void RansacHomography( CHomography& homo, 
					  const MatchArray& aryMatch, 
					  const CFeatureArray& set1, 
					  const CFeatureArray& set2, 
					  float inlierTol, int numIter );

int main(int argc, char* argv[])
{
	int i = -1;
	int j = -1;
	float inlierTol = 1;
	int numIter = 1000;

	// ransac
	srand( time(NULL) );

	int arg = 0;
	while( ++arg < argc) 
	{ 
		if( !strcmp(argv[arg], "-i") )
			i = atoi( argv[++arg] );

		if( !strcmp(argv[arg], "-j") )
			j = atoi( argv[++arg] );

		if( !strcmp(argv[arg], "-tol") )
			inlierTol = atof( argv[++arg] );

		if( !strcmp(argv[arg], "-iter") )
			numIter = atoi( argv[++arg] );
	}	

	try
	{
		char strBuf[128];

		// Step 1: load the extracted features
		CFeatureArray set_i, set_j;
		
		sprintf( strBuf, "%04d.key", i );
		LoadSiftFromFile( set_i, strBuf );
		
		sprintf( strBuf, "%04d.key", j );
		LoadSiftFromFile( set_j, strBuf );

		// Step 2: load the matches
		MatchArray aryMatch;
		LoadMatchArray( aryMatch, i, j );

		// Step 3: Estimate the homography
		CHomography homo;
		RansacHomography( homo, aryMatch, set_i, set_j, inlierTol, numIter );

		// Step 4: Save the homography
		SaveHomography( homo, i, j );
	}
	catch( exception& err )
	{
		printf( "%s\n", err.what() );
	}

	return 0;
}

bool isCollinear(const MatchArray& aryMatch, 
				 const CFeatureArray& set1, 
				 const CFeatureArray& set2, 
				 int indices[])
{
	CvPoint2D64f pts[4];
	for(int i = 0; i < 4; i++)
	{
		pts[i].x = set1[aryMatch[indices[i]].first]->x;
		pts[i].y = set1[aryMatch[indices[i]].first]->y;
	}

	int testCases[4][3] = {
							{0,1,2},
							{0,1,3},
							{0,2,3},
							{1,2,3}
							};

	for(int i = 0; i < 4; i++)
	{
		if(((pts[testCases[i][0]].x -  pts[testCases[i][1]].x) == 0) && ((pts[testCases[i][2]].x -  pts[testCases[i][1]].x) == 0))
			return true;
		if(((pts[testCases[i][0]].x -  pts[testCases[i][1]].x) == 0) || ((pts[testCases[i][2]].x -  pts[testCases[i][1]].x) == 0))
			return false;
		if((((pts[testCases[i][0]].y -  pts[testCases[i][1]].y)/((pts[testCases[i][0]].x -  pts[testCases[i][1]].x))) ==
			((pts[testCases[i][2]].y -  pts[testCases[i][1]].y)/((pts[testCases[i][2]].x -  pts[testCases[i][1]].x)))))
			return true;
	}

	return false;
}

/**
 *	RansacHomography:
 *		Input:
 *			aryMatch - an array of potential matches between two images
 *			inlierTol - the tolerance to regard a match as an inlier
 *			numIter - number of iterations for Ransac
 *
 *		Ouput:
 *			homo - the best estimated homography (with the max nubmer of inliers)
 */
void RansacHomography( CHomography& homo, 
					  const MatchArray& aryMatch, 
					  const CFeatureArray& set1, 
					  const CFeatureArray& set2, 
					  float inlierTol, int numIter )
{
    std::vector<int> RansacMatches, RansacMatches_;

	const float SQR_TOL = inlierTol*inlierTol;
	const int NUM_SAMP = 6;
	int maxInlier = -1;

	double dA[ NUM_SAMP*2*8 ];
	double dB[ NUM_SAMP*2 ];
	
	// ToDo2: Find homography using RANSAC

	for(int i = 0 ; i < numIter;)
	{
		int randIdx[4];
		int arraySize = aryMatch.size();
		for(int j = 0; j < 4;)
		{
			randIdx[j] = rand() % arraySize;
			bool skip = false;
			for(int j1 = 0; j1 < j; j1++)
			{
				if(randIdx[j] == randIdx[j1])
					skip = true;
			}
			if(!skip)
					j++;
		}

		if(isCollinear(aryMatch, set1, set2,randIdx))
			continue;

        double matrixA[8][8] = {0};
        double matrixB[8] = {0};

        for(int j = 0; j < 4;)
        {
            CvPoint2D64f pt1, pt2;
            pt1.x = set1[aryMatch[randIdx[j]].first]->x;
            pt1.y = set1[aryMatch[randIdx[j]].first]->y;

            pt2.x = set2[aryMatch[randIdx[j]].second]->x;
            pt2.y = set2[aryMatch[randIdx[j]].second]->y;

            matrixA[j*2][0] = pt1.x;
            matrixA[j*2][1] = pt1.y;
            matrixA[j*2][2] = 1;
            matrixA[j*2][6] = -pt2.x * pt1.x;
            matrixA[j*2][7] = -pt2.x * pt1.y;

            matrixA[j*2+1][3] = pt1.x;
            matrixA[j*2+1][4] = pt1.y;
            matrixA[j*2+1][5] = 1;
            matrixA[j*2+1][6] = -pt2.y * pt1.x;
            matrixA[j*2+1][7] = -pt2.y * pt1.y;

            matrixB[j*2] = pt2.x;
            matrixB[j*2+1] = pt2.y;
            j++;
        }

        cv::Mat matA = cv::Mat(8,8,CV_64FC1, matrixA);
        cv::Mat matB = cv::Mat(8,1,CV_64FC1, matrixB);
        cv::Mat matx = cv::Mat(8,1,CV_64FC1);
        solve(matA, matB, matx,cv::DECOMP_SVD);

        int matchSize = aryMatch.size();
        std::vector<cv::Point2f> A_vec(matchSize);
        std::vector<cv::Point2f> B_vec(matchSize);
        std::vector<cv::Point2f> transformed_vec(matchSize);
        
        for(int j = 0; j < matchSize; j++)
        {
            A_vec[j].x = set1[aryMatch[j].first]->x;
            A_vec[j].y = set1[aryMatch[j].first]->y;
            

            B_vec[j].x = set2[aryMatch[j].second]->x;
            B_vec[j].y = set2[aryMatch[j].second]->y;
        }

        double* matxData = (double*)(matx.data);
        double transform[3][3];
        memcpy(transform, matxData, sizeof(double) * 8);
        transform[2][2]  = 1;
        cv::Mat matTransform = cv::Mat(3,3, CV_64F, transform);

        cv::Mat matFullTransformed = cv::Mat(matchSize,2,CV_64F);
        cv::perspectiveTransform(A_vec, transformed_vec, matTransform);

        int nInliers = 0;
        for(int j = 0; j < matchSize; j++)
        {
            cv::Point2f dist = (B_vec[j] - transformed_vec[j]);
            if((dist.x * dist.x + dist.y * dist.y) <= inlierTol*inlierTol)
            {
                RansacMatches_.push_back(j);
                nInliers++;
            }
        }

        if(maxInlier < nInliers)
        {
            RansacMatches.clear();
            RansacMatches.insert(RansacMatches.begin(), RansacMatches_.begin(), RansacMatches_.end());
            maxInlier = nInliers;
            homo.writeHomography((double*)matTransform.data);
        }
        RansacMatches_.clear();
		i++;
	}
	/*FILE* file = fopen("RansacMatches.txt", "w");
    std::vector<int>::iterator it, Rend =  RansacMatches.end();
    for(it= RansacMatches.begin(); it!=Rend; it++)
    fprintf(file, "%d\n", *it);
    fclose(file);*/


    {
        double* matrixA = (double*) calloc(maxInlier * 2 * 8, sizeof(double));
        double* matrixB = (double*) calloc(maxInlier * 2, sizeof(double));

        for(int j = 0; j < maxInlier;)
        {
            CvPoint2D64f pt1, pt2;
            pt1.x = set1[aryMatch[RansacMatches[j]].first]->x;
            pt1.y = set1[aryMatch[RansacMatches[j]].first]->y;

            pt2.x = set2[aryMatch[RansacMatches[j]].second]->x;
            pt2.y = set2[aryMatch[RansacMatches[j]].second]->y;

            matrixA[j*2 * 8 + 0] = pt1.x;
            matrixA[j*2 * 8 + 1] = pt1.y;
            matrixA[j*2 * 8 + 2] = 1;
            matrixA[j*2 * 8 + 6] = -pt2.x * pt1.x;
            matrixA[j*2 * 8 + 7] = -pt2.x * pt1.y;

            matrixA[(j*2+1) * 8 + 3] = pt1.x;
            matrixA[(j*2+1) * 8 + 4] = pt1.y;
            matrixA[(j*2+1) * 8 + 5] = 1;
            matrixA[(j*2+1) * 8 + 6] = -pt2.y * pt1.x;
            matrixA[(j*2+1) * 8 + 7] = -pt2.y * pt1.y;

            matrixB[j*2] = pt2.x;
            matrixB[j*2+1] = pt2.y;
            j++;
        }

        double htemp[9] = {0};
        cv::Mat matA = cv::Mat(maxInlier*2,8,CV_64FC1, matrixA);
        cv::Mat matB = cv::Mat(maxInlier*2,1,CV_64FC1, matrixB);
        cv::Mat matx = cv::Mat(maxInlier*2,1,CV_64FC1);
        solve(matA, matB, matx,CV_SVD);
        double* matxData = (double*)(matx.data);
        memcpy(htemp, matxData, sizeof(double) * 8);
        htemp[8]  = 1;
        homo.writeHomography((double*)htemp);

        free(matrixA);
        free(matrixB);
    }

	printf( "homography inliers: %d(%d)\n", maxInlier, aryMatch.size());
}