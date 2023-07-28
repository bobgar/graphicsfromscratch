#include "Scene.h"

Scene::Scene(std::vector<sphere> &spheres, std::vector<light>& lights)
	: spheres(spheres) , lights(lights)
	{}

Scene Scene::MakeTestScene() {
	std::vector<sphere> spheres = std::vector<sphere>(4);
	spheres[0] = sphere{ vec3{0,-1,3}, 1, color{255,0,0,255}, 500 , .2 };
	spheres[1] = sphere{ vec3{2,0,4}, 1, color{0,0,255,255}, 500, .3 };
	spheres[2] = sphere{ vec3{-2,0,4}, 1, color{0,255,0,255}, 10, .4 };
	spheres[3] = sphere{ vec3{0,-5001,0}, 5000, color{255,255,0,255}, 1000, .5 };
	std::vector<light> lights = std::vector<light>(3);
	lights[0] = light{ AMBIENT, 0.2 };
	lights[1] = light{ POINT, 0.6, vec3{2,1,0} };
	lights[2] = light{ DIRECTIONAL, 0.2, vec3{}, vec3{1,4,4} };
	return Scene(spheres, lights);
}