siftWin32 <IMG_0001.pgm> 0001.key
siftWin32 <IMG_0002.pgm> 0002.key
siftWin32 <IMG_0003.pgm> 0003.key
siftWin32 <IMG_0004.pgm> 0004.key
siftWin32 <IMG_0005.pgm> 0005.key
siftWin32 <IMG_0006.pgm> 0006.key
siftWin32 <IMG_0007.pgm> 0007.key
siftWin32 <IMG_0008.pgm> 0008.key
siftWin32 <IMG_0009.pgm> 0009.key
siftWin32 <IMG_0010.pgm> 0010.key
siftWin32 <IMG_0011.pgm> 0011.key
siftWin32 <IMG_0012.pgm> 0012.key
siftWin32 <IMG_0013.pgm> 0013.key
siftWin32 <IMG_0014.pgm> 0014.key
siftWin32 <IMG_0015.pgm> 0015.key
siftWin32 <IMG_0016.pgm> 0016.key
siftWin32 <IMG_0017.pgm> 0017.key

MatchPair -i 1 -j 2
MatchPair -i 2 -j 3
MatchPair -i 3 -j 4
MatchPair -i 4 -j 5
MatchPair -i 5 -j 6
MatchPair -i 6 -j 7
MatchPair -i 7 -j 8
MatchPair -i 8 -j 9
MatchPair -i 9 -j 10
MatchPair -i 10 -j 11
MatchPair -i 11 -j 12
MatchPair -i 12 -j 13
MatchPair -i 13 -j 14
MatchPair -i 14 -j 15
MatchPair -i 15 -j 16
MatchPair -i 16 -j 17

AlignPair -i 1 -j 2 -tol 1.5 -iter 5000
AlignPair -i 2 -j 3 -tol 1.5 -iter 5000
AlignPair -i 3 -j 4 -tol 1.5 -iter 5000
AlignPair -i 4 -j 5 -tol 1.5 -iter 5000
AlignPair -i 5 -j 6 -tol 1.5 -iter 5000
AlignPair -i 6 -j 7 -tol 1.5 -iter 5000
AlignPair -i 7 -j 8 -tol 1.5 -iter 5000
AlignPair -i 8 -j 9 -tol 1.5 -iter 5000
AlignPair -i 9 -j 10 -tol 1.5 -iter 5000
AlignPair -i 10 -j 11 -tol 1.5 -iter 5000
AlignPair -i 11 -j 12 -tol 1.5 -iter 5000
AlignPair -i 12 -j 13 -tol 1.5 -iter 5000
AlignPair -i 13 -j 14 -tol 1.5 -iter 5000
AlignPair -i 14 -j 15 -tol 1.5 -iter 5000
AlignPair -i 15 -j 16 -tol 1.5 -iter 5000
AlignPair -i 16 -j 17 -tol 1.5 -iter 5000

CameraPose -ib 1 -ie 17 -u 225 -v 384 -alpha 1 -tol 2

WarpCylinderical -ib 1 -ie 17