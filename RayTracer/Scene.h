#pragma once
#include "CommonStructs.h"
#include <vector>

class Scene {

public:
	Scene(std::vector<sphere>& spheres, std::vector <light> & lights);
	//~Scene();
	static Scene MakeTestScene();
	std::vector<sphere> spheres;
	std::vector<light> lights;
};