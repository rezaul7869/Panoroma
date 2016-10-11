// WarpCylinderical.cpp : Defines the entry point for the console application.

#include "homography.h"
#include "Camera.h"

#include <cv.h>
#include <highgui.h>

#define _USE_MATH_DEFINES
#include <cmath>

#define M_PI       3.14159265358979323846
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#define max(a,b)            (((a) > (b)) ? (a) : (b))

void getColor(IplImage *img, char* color, CvPoint2D64f pt)
{
    char* data = img->imageData;
    int loc =  (abs(pt.y) * img->width + abs(pt.x)) * img->nChannels;
    color[0] = data[loc];
    color[1] = data[loc+1];
    color[2] = data[loc+2];

}

int main(int argc, char* argv[])
{
	// the size of the the panorama image
	CvSize szPano = cvSize( 8000, 600 );

	// the radius
	const double r = (double)szPano.width / (2*M_PI);
	
	// the pan angle per pixel
	const double angPixel =  2*M_PI / (double)szPano.width;

	// the number of images in the sequence
	int ib = 0;
	int ie = 0;
	int numImage = 0;

	int arg = 0;
	while( ++arg < argc) 
	{
		if( !strcmp(argv[arg], "-ib") )
			ib = atoi( argv[++arg] );

		if( !strcmp(argv[arg], "-ie") )
			ie = atoi( argv[++arg] );
	}	

	// the indices of the image sequence
	numImage = ie-ib+1;
	int* imgId = new int[ numImage ];
	for( int i=0; i<numImage; ++i )
		imgId[i] = ib+i;

	// the output panorama
	IplImage* imgPano = cvCreateImage( szPano, IPL_DEPTH_32F, 3 );
	cvSetZero( imgPano );
	
	for( int i=0; i<numImage; ++i ) // for each image
	{
		// load the image
		char strName[128];
		sprintf( strName, "IMG_%04d.JPG", imgId[i] ); // modify according to your image name
		IplImage* img = cvLoadImage( strName, 1 );

		// load the camera
		CCamera cam;
		LoadCamera( cam, imgId[i] );

		//
		double dir[3];
		
		CvPoint2D64f corner[4] = { 
			cvPoint2D64f( 0, 0 ),
			cvPoint2D64f( 0, img->height ),
			cvPoint2D64f( img->width, 0 ),
			cvPoint2D64f( img->width, img->height ) };
		
		int lm = INT_MAX; // left most
		int rm = -INT_MAX; // right most

		// find the left end of the image in the panorama
		for( int j=0; j<2; ++j )
		{
			cam.GetRayDirectionWS( dir, corner[j] );			
			double theta = atan2( dir[0], dir[2] );

			lm = min( lm, floor( szPano.width *theta /(2*M_PI) ) );
		}

		// find the right end of the image in the panorama
		for( int j=0; j<2; ++j )
		{
			cam.GetRayDirectionWS( dir, corner[j+2] );			
			double theta = atan2( dir[0], dir[2] );

			rm = max( rm, ceil( szPano.width *theta /(2*M_PI) ) );
		}

		if( lm > rm ) // 180 degree crossing
			lm -= szPano.width;

		printf( "L:%d, R:%d\n", lm, rm );

        if(i==0)
        {
            for(int x = 0; x < img->width;x++)
            {
                for(int y = 0; y < img->height && y < imgPano->height;y++)
                {
                    //char color[3];
                    //CvPoint2D64f cpt;
                    //cpt.x = x;
                    //cpt.y = y;
                    //getColor(img, color, cpt);
                    //char* PanImgData =  imgPano->imageData;
                    //int index =  ((x * imgPano->width) + y) * imgPano->nChannels;
                    //PanImgData[index] = color[0];
                    //PanImgData[index+1] = color[1];
                    //PanImgData[index+2] = color[2];
                            //CvScalar s;
                            //s=cvGet2D(img,y,x);         
                            //cvSet2D(imgPano,y,x,s);
                }
            }
        }

		// ToDo 5: cylinderical blending
		for( int m=0; m<szPano.height; ++m )
        {
			for( int n=lm; n<rm; ++n )
			{
				// 1. project pixel in imgPano(m,n) to the virtual cylinder
                CvPoint3D64f wpt;
				double theta = atan2(n*1.0,m);
				int y = n;
				if(n < 0) y = 8000 + n;
				wpt.x = r*sin(y*2*M_PI/szPano.width);
                wpt.y = m;
                wpt.z = r*cos(y*2*M_PI/szPano.width);
                
                
				// 2. project it back to img to check if it is in front of the camera

                CvPoint2D64f cpt = cam.GetProjection(wpt);
				// 3. find its location in img, and assign its RGB value to imgPano(m,n)

                if((cpt.x > 0 && cpt.x < img->width) &&
                    (cpt.y > 0 && cpt.y < img->height))
                {
                    /*char color[3];
                    getColor(img, color, cpt);
                    char* PanImgData =  imgPano->imageData;
                    int index =  ((m * imgPano->width) + n) * imgPano->nChannels;
                    PanImgData[index] = color[0];
                    PanImgData[index+1] = color[1];
                    PanImgData[index+2] = color[2];*/
                    CvScalar s;
                    s=cvGet2D(imgPano,m,y);   
                    //if(s.val[0] == 0 && s.val[1] == 0 && s.val[2] ==0 && s.val[3]==0)
                    {
                        s=cvGet2D(img,cpt.y,cpt.x);         
                        cvSet2D(imgPano,m,y,s);
                    }
                }
			}
        }
	}
	
	cvSaveImage( "panorama.jpg", imgPano );
	cvReleaseImage( &imgPano );

	return 0;
}

