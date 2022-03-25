#ifndef __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_ZBUFFER_H_
#define __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_ZBUFFER_H_

#include <vector>

class ZBuffer: public std::vector<std::vector<double>>
{
  public:
	//Constructor: maakt een Z-Buffer van de correcte
	//grootte aan en initialiseert alle velden op +inf
	ZBuffer(const int width, const int height);
};

#endif // __PROJECTS_GRAPHICS_ENGINE_CPP_UTIL_ZBUFFER_H_