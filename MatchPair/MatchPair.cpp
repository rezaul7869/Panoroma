// MatchPair.cpp : Defines the entry point for the console application.

#include "SiftFeature.h"
#include <cv.h>
#include <highgui.h>
#include <unordered_set>

int main(int argc, char* argv[])
{
    FILE* file = fopen("RansacMatches.txt", "r");
    std::unordered_set<int> RansacMatches;
    int inp = -1;
    while(fscanf(file, "%d\n", &inp)!=EOF)
    {
        RansacMatches.insert(inp);
    }
    fclose(file);

	int i = -1;
	int j = -1;

	int arg = 0;
	while( ++arg < argc) 
	{ 
		if( !strcmp(argv[arg], "-i") )
			i = atoi( argv[++arg] );

		if( !strcmp(argv[arg], "-j") )
			j = atoi( argv[++arg] );
	}	

	try
	{
		CFeatureArray set_i, set_j;

		// Step 1: load the extracted features using LoadSiftFromFile()
		char strBuf[128];
		sprintf( strBuf, "%04d.key", i );
		LoadSiftFromFile( set_i, strBuf );
		
		sprintf( strBuf, "%04d.key", j );
		LoadSiftFromFile( set_j, strBuf );

		// Step 2: match features
		MatchArray aryMatch;
		MatchSiftFeatures( aryMatch, set_i, set_j );
        /////////////////////////////////////////////////////////////////
        sprintf( strBuf, "IMG_%04d.JPG", i );
        cv::Mat img_1 = cv::imread(strBuf);
        sprintf( strBuf, "IMG_%04d.JPG", j );
        cv::Mat img_2 = cv::imread(strBuf);
        
        std::vector<cv::KeyPoint> keypoints_1(set_i.size());
        std::vector<cv::KeyPoint> keypoints_2(set_j.size());
        int set_i_length= set_i.size();
        int set_j_length= set_j.size();
        for(int i = 0; i < set_i_length; i++)
        {
            keypoints_1[i].pt = cv::Point2f(set_i[i]->x,set_i[i]->y);
            keypoints_1[i].size = 1;
        }

        for(int j = 0; j < set_j_length; j++)
        {
            keypoints_2[j].pt = cv::Point2f(set_j[j]->x,set_j[j]->y);
            keypoints_2[j].size = 1;
        }
        int aryMatchSize = aryMatch.size();
        std::vector<cv::DMatch> good_matches;
        for(int i = 0; i < aryMatchSize; i++)
        {
            if(RansacMatches.find(i)!= RansacMatches.end())
                good_matches.push_back(cv::DMatch(aryMatch[i].second,aryMatch[i].first, 5));
        }

        cv::Mat img_matches;
        drawMatches( img_2, keypoints_2, img_1, keypoints_1,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //-- Show detected matches
        //imshow( "Good Matches", img_matches );
        /////////////////////////////////////////////////////////////////
		// Step 3: Save the matches
		SaveMatchArray( aryMatch, i, j );
	}
	catch( exception& err )
	{
		printf( "%s\n", err.what() );
	}
    cv::waitKey(0);
	return 0;
}

