#ifndef JSON_READER
#define JSON_READER
#include <string>
// JSON Reader
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <cstdio>

class JSONReader
{
public:
	JSONReader();
	~JSONReader();
	static rapidjson::Document readJSON(const std::string& filepath)
	{
	    FILE* fp = fopen(filepath.c_str(), "r"); // non-Windows use "r"
	    assert(fp != NULL);
	    char readBuffer[65536];
	    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
	    rapidjson::Document d;
	    d.ParseStream(is);
	    fclose(fp);
	    return d;
	}
};

#endif