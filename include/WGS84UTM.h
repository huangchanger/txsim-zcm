#pragma once
/*
Acknowledgement:http://www.cnblogs.com/wb-DarkHorse/archive/2013/06/26/3156212.html
*/


class UTMCoor
{
public:
	double x;
	double y;
};

class WGS84Corr
{
public:
	double lat;
	double lon;
};

class Point2d
{
public:
    Point2d();
    Point2d(double x,double y);
    Point2d operator+(const Point2d& pt);
	Point2d operator-(const Point2d& pt);
	Point2d operator/(const double& div);
    friend Point2d operator*(const Point2d& pt, const double multiplier);
    friend Point2d operator*(const double multiplier, const Point2d& pt);
	friend void CoorRotate(Point2d& pt, const double theta);

	double norm();
    float x;
    float y;
};
Point2d operator*(const Point2d& pt, const double multiplier);
Point2d operator*(const double multiplier, const Point2d& pt);
void CoorRotate(Point2d& pt, const double theta);

/*
* DegToRad
*
* Converts degrees to radians.
*
*/
inline double DegToRad(double deg);

/*
* RadToDeg
*
* Converts radians to degrees.
*
*/
inline double RadToDeg(double rad);
/*
* ArcLengthOfMeridian
*
* Computes the ellipsoidal distance from the equator to a point at a
* given latitude.
*
* Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
* GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
*
* Inputs:
*     phi - Latitude of the point, in radians.
*
* Globals:
*     sm_a - Ellipsoid model major axis.
*     sm_b - Ellipsoid model minor axis.
*
* Returns:
*     The ellipsoidal distance of the point from the equator, in meters.
*
*/
double ArcLengthOfMeridian(double phi);

/*
* UTMCentralMeridian
*
* Determines the central meridian for the given UTM zone.
*
* Inputs:
*     zone - An integer value designating the UTM zone, range [1,60].
*
* Returns:
*   The central meridian for the given UTM zone, in radians, or zero
*   if the UTM zone parameter is outside the range [1,60].
*   Range of the central meridian is the radian equivalent of [-177,+177].
*
*/
inline double UTMCentralMeridian(int zone);


/*
* FootpointLatitude
*
* Computes the footpoint latitude for use in converting transverse
* Mercator coordinates to ellipsoidal coordinates.
*
* Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
*   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
*
* Inputs:
*   y - The UTM northing coordinate, in meters.
*
* Returns:
*   The footpoint latitude, in radians.
*
*/
double FootpointLatitude(double y);
/*
* MapLatLonToXY
*
* Converts a latitude/longitude pair to x and y coordinates in the
* Transverse Mercator projection.  Note that Transverse Mercator is not
* the same as UTM; a scale factor is required to convert between them.
*
* Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
* GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
*
* Inputs:
*    phi - Latitude of the point, in radians.
*    lambda - Longitude of the point, in radians.
*    lambda0 - Longitude of the central meridian to be used, in radians.
*
* Outputs:
*    xy - A 2-element array containing the x and y coordinates
*         of the computed point.
*
* Returns:
*    The function does not return a value.
*
*/
void MapLatLonToXY(double phi, double lambda, double lambda0, UTMCoor &xy);



/*
* MapXYToLatLon
*
* Converts x and y coordinates in the Transverse Mercator projection to
* a latitude/longitude pair.  Note that Transverse Mercator is not
* the same as UTM; a scale factor is required to convert between them.
*
* Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
*   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
*
* Inputs:
*   x - The easting of the point, in meters.
*   y - The northing of the point, in meters.
*   lambda0 - Longitude of the central meridian to be used, in radians.
*
* Outputs:
*   philambda - A 2-element containing the latitude and longitude
*               in radians.
*
* Returns:
*   The function does not return a value.
*
* Remarks:
*   The local variables Nf, nuf2, tf, and tf2 serve the same purpose as
*   N, nu2, t, and t2 in MapLatLonToXY, but they are computed with respect
*   to the footpoint latitude phif.
*
*   x1frac, x2frac, x2poly, x3poly, etc. are to enhance readability and
*   to optimize computations.
*
*/
void MapXYToLatLon(double x, double y, double lambda0, WGS84Corr &philambda);


/*
* LatLonToUTMXY
*
* Converts a latitude/longitude pair to x and y coordinates in the
* Universal Transverse Mercator projection.
*
* Inputs:
*   lat - Latitude of the point, in degree.
*   lon - Longitude of the point, in degree.
*   zone - UTM zone to be used for calculating values for x and y.
*          If zone is less than 1 or greater than 60, the routine
*          will determine the appropriate zone from the value of lon.
*
* Outputs:
*   xy - A 2-element array where the UTM x and y values will be stored.
*
* Returns:
*   void
*
*/
void LatLonToUTMXY(double lat, double lon, UTMCoor &xy,  int zone = 30);

/*
* UTMXYToLatLon
*
* Converts x and y coordinates in the Universal Transverse Mercator
* projection to a latitude/longitude pair.
*
* Inputs:
*    x - The easting of the point, in meters.
*    y - The northing of the point, in meters.
*    zone - The UTM zone in which the point lies.
*    southhemi - True if the point is in the southern hemisphere;
*               false otherwise.
*
* Outputs:
*    latlon - A 2-element array containing the latitude and
*            longitude of the point, in radians.
*
* Returns:
*    The function does not return a value.
*
*/
void UTMXYToLatLon(double x, double y,  WGS84Corr &latlon, int zone = 30, bool southhemi = false);