#include <iostream>
#include <fstream>
#include <string>
using namespace std;

// a simple counting program to calculate aabb size
int main(int argc, char* argv[])
{
	if(argc == 2)
	{
		ifstream fin(argv[1]);
		
		string tmp;
		float x, y, z;
		float xmin, xmax, ymin, ymax, zmin, zmax;
		xmin = ymin = zmin = 10000.0f;
		xmax = ymax = zmax = -10000.0f;
		fin >> tmp;
		while(tmp != "v")
		{
			fin >> tmp;
		}
		while(tmp == "v")
		{
			fin >> x >> y >> z;
			if(x < xmin) xmin = x;
			if(y < ymin) ymin = y;
			if(z < zmin) zmin = z;

			if(x > xmax) xmax = x;
			if(y > ymax) ymax = y;
			if(z > zmax) zmax = z;
			fin >> tmp;
		}
		cout << xmin << " " << xmax << " " << ymin << " " << ymax << " " << zmin << " " << zmax << endl;
	}
	return 0;
}