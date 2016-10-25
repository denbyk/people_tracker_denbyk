/*
 * Denoiser.h
 *
 *  Created on: 12 ott 2016
 *      Author: denbyk
 */

#ifndef INCLUDE_TRACKINGDENOISER_H_
#define INCLUDE_TRACKINGDENOISER_H_

#include <vector>

struct TrackedPoint
{
	double x, y, z;
};

class TrackingDenoiser {
	std::vector<TrackedPoint> trackings;
	int size;
	int index;
public:

	TrackingDenoiser();

	TrackingDenoiser(int size);

	void addTracking(TrackedPoint p);

	TrackedPoint getAvg();

	//virtual ~TrackingDenoiser();
};

#endif /* INCLUDE_TRACKINGDENOISER_H_ */
