#include <fstream>
#include <iostream>
#define MAX_NUM 50
#define SAMPLE_STEP 5
using namespace std;

void createObj(void)
{
	ofstream fout("test.obj");
	double left = -3.0, right = 3.0;
	double interval = (right - left) / MAX_NUM;

	if(!fout.is_open())
		cout << ".obj cannot open" << endl;

	fout << "mtllib test.mtl" << endl;

	for(int i = 0; i < MAX_NUM; i++)
	{
		fout << "v " << left + i * interval << " 0.0 0.0" << endl;
	}
	fout.close();
}

void createSample(void)
{
	ofstream fout("test.sample");
	double left = -3.0, right = 3.0;
	double interval = SAMPLE_STEP * (right - left) / MAX_NUM;

	if(!fout.is_open())
		cout << ".sample cannot open" << endl;

	fout << "mtllib test.mtl" << endl;

	for(int i = 0; i < MAX_NUM / SAMPLE_STEP; i++)
	{
		fout << "v " << left + i * interval << " 0.0 0.0" << endl;
	}
	fout.close();
}

int main(void)
{
	createObj();
	createSample();
}