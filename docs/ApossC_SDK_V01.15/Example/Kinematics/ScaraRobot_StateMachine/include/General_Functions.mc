/*********************************************************************
** General Functions
**
*********************************************************************/
#include "General_Functions.mh"

/*
** Initialize all user parameters used by the user dialog.
*/
void UserParamInit(void)
{
    long scale = USER_FACTOR_SCALE;     // Factors and positions are in 1/10000.

    USER_PARAM(USR_COMMAND) = 0;

	USER_PARAM(USR_ARRNO) 		= 0;        // Zero-based.
	USER_PARAM(USR_PATHNO) 		= 0;        // Zero-based.
	USER_PARAM(USR_CYCLECMD) 	= 10;
	USER_PARAM(USR_SCALE) 		= scale;

	USER_PARAM(USR_MOVEVEL) 	= 250;
	USER_PARAM(USR_MOVEACC) 	= 200;
	USER_PARAM(USR_MOVEJERK) 	= 2000;

	USER_PARAM(USR_MACHPOSX) 	=  400*scale;
	USER_PARAM(USR_MACHPOSY) 	=  0*scale;
	USER_PARAM(USR_MACHPOSZ) 	=  -260*scale;
	USER_PARAM(USR_WORKPOSX) 	=  200*scale;
	USER_PARAM(USR_WORKPOSY) 	=  400*scale;
	USER_PARAM(USR_WORKPOSZ) 	=  75*scale;

	USER_PARAM(USR_PATHROTXYD) 	= 0;
	USER_PARAM(USR_PATHROTYZD) 	= 0;
	USER_PARAM(USR_PATHROTZXD) 	= 0;
	USER_PARAM(USR_PATHSTRCHX) 	= scale;
	USER_PARAM(USR_PATHSTRCHY) 	= scale;
	USER_PARAM(USR_PATHSTRCHZ) 	= scale;
	USER_PARAM(USR_PATHSCALE) 	= scale;
	USER_PARAM(USR_PATHVEL) 	= 500;
	USER_PARAM(USR_PATHACC) 	= 500;
	USER_PARAM(USR_PATHJERK) 	= 4000;
	USER_PARAM(USR_PATHSTTYPE) 	= 0x02;
	USER_PARAM(USR_PATHSTARTMP) = 0;
}

/*
** This function exists so that the DimArray() function can be used
** to select the DIM array that is being used to hold the Path array.
*/
void DoPathStart(long patharray[], long pathno, long masterpos, long flags)
{
    PathStart(Robot, patharray, pathno, PathToWork, masterpos, flags);
}

/*
** Given a curve number and a segment number within that curve,
** return the segment code of the segment.  This is where the
** pen up/down information is stored.
*/
long GetSegmentCode(long patharray[], long crvnum, long segnum)
{
    long segndx = PathArrayIndex(PATH_INDEX_SEGMENT, patharray, crvnum, segnum);
    long segtyp = (patharray[segndx+PTH_SEG_TYPE]);
    long segcod = PathSegCode(segtyp);
	#if 0
		print(" segnum = ", segnum);
		print(" segndx = ", segndx);
		print(" segtyp = ", segtyp);
		print(" segcod = ", segcod);
	#endif

    return(segcod);
}

/*
** Given a curve number, return the number of segments in the
** curve.
*/
long GetSegmentCount(long patharray[], long crvnum)
{
    long crvndx = PathArrayIndex(PATH_INDEX_CURVE, patharray, crvnum);
    long segcnt = patharray[crvndx+PTH_INF_FIXCNT];

	#if 0
		print (" crvnum = ", crvnum);
		print (" segcnt = ", segcnt);
	#endif

    return(segcnt);
}
