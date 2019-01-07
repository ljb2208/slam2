#pragma once

#include <sstream>
#include <fstream>
#include <dirent.h>

#include <boost/thread.hpp>

inline int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
    	std::string name = std::string(dirp->d_name);

    	if(name != "." && name != "..")
    		files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++)
	{
		if(files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

    return files.size();
}

inline void split(const std::string& src, const std::string& delim, std::vector<std::string>& dest)
{
    std::string str = src;
    std::string::size_type start = 0, index;
    std::string substr;

    index = str.find_first_of(delim, start);    //在str中查找(起始：start) delim的任意字符的第一次出现的位置
//    while(index != std::string::npos)
    while(1)
    {
        substr = str.substr(start, index-start);
        dest.push_back(substr);
        start = str.find_first_not_of(delim, index);    //在str中查找(起始：index) 第一个不属于delim的字符出现的位置
        if(start == std::string::npos) return;

        index = str.find_first_of(delim, start);
    }
}

class ImageFolderReader{

public:
    ImageFolderReader(std::string path, std::string calibFile);
    ~ImageFolderReader();

    int getNumImages();
    std::string getImageFilename(int index);
    double getTimestamp(int index);

private:
    void loadTimestamps();
    std::string path;
    std::string calibFile;

    std::vector<std::string> files;
    std::vector<double> timestamps;
	std::vector<float> exposures;

};


