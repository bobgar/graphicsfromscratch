#pragma once
#include "CommonStructs.h"

inline matrix4 MakeIdentityMatrix() {
	return matrix4{ 1,0,0,0,
					 0,1,0,0,
					 0,0,1,0,
					 0,0,0,1 };
}

inline matrix4 MakeRotationMatrix(float yaw, float pitch, float roll) {
	float cy = cos(yaw);
	float sy = sin(yaw);
	float cp = cos(pitch);
	float sp = sin(pitch);
	float cr = cos(roll);
	float sr = sin(roll);
	return matrix4{ cy * cp , cy * sp * sr - sp * cr , cy * sp * cr + sp * sr , 0,
					  sy * cp , sy * sp * sr + cy * cr , sy * sp * cr - cy * sr , 0,
					  -sp     , cp * sr                , cp * cr                , 0,
					  0       ,0                       ,0                       , 1 };
}

inline matrix4 MakeTranslationMatrix(vec3 t) {
	return matrix4{ 1,0,0,t.x,
					  0,1,0,t.y,
					  0,0,1,t.z,
					  0,0,0,1 };
}

inline matrix4 MakeScalingMatrix(float sx, float sy, float sz) {
	return matrix4{ sx,0 ,0 ,0,
					  0 ,sy,0 ,0,
					  0 ,0 ,sz,0,
					  0 ,0 ,0 ,1 };
}

