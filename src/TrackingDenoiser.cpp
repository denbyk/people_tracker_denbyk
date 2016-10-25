/*
 * Denoiser.cpp
 *
 *  Created on: 12 ott 2016
 *      Author: denbyk
 */

#include <TrackingDenoiser.h>
#include <stdexcept>
#include <iostream>

TrackingDenoiser::TrackingDenoiser()
{
	this->size = 0;
	this->index = 0;
}


TrackingDenoiser::TrackingDenoiser(int size)
{
	this->size = size;
	this->index = 0;
}


void TrackingDenoiser::addTracking(TrackedPoint p)
{
	try
	{
		trackings.at(index) = p;
	}
	catch(const std::out_of_range& e)
	{
		trackings.push_back(p);
		//std::cout << p.x << "\n";
	}

	index = (index + 1) % this->size;

}

TrackedPoint TrackingDenoiser::getAvg()
{
	double xtot = 0;
	double ytot = 0;
	double ztot = 0;
	//std::cout << trackings.size();
	for (int i = 0; i < trackings.size(); i++)
	{
		xtot+=trackings[i].x;
		ytot+=trackings[i].y;
		ztot+=trackings[i].z;
		//std::cout << xtot << " " << ytot << " " << ztot << "\n";
	}
	TrackedPoint p;
	p.x = xtot / double(trackings.size());
	p.y = ytot / double(trackings.size());
	p.z = ztot / double(trackings.size());
	return p;
}
