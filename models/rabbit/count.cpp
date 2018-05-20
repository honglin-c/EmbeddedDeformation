#include <iostream>
using namespace std;

// a simple counting program to calculate aabb size
int main()
{
	char tmp;
	float x, y, z;
	float xmin, xmax, ymin, ymax, zmin, zmax;
	xmin = ymin = zmin = 1000.0f;
	xmax = ymax = zmax = -1000.0f;
	cin >> tmp;
	while(tmp == 'v')
	{
		cin >> x >> y >> z;
		if(x < xmin) xmin = x;
		if(y < ymin) ymin = y;
		if(z < zmin) zmin = z;

		if(x > xmax) xmax = x;
		if(y > ymax) ymax = y;
		if(z > zmax) zmax = z;
		cin >> tmp;
	}
	cout << xmin << " " << xmax << " " << ymin << " " << ymax << " " << zmin << " " << zmax << endl;
	return 0;
}