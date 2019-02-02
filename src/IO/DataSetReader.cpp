#include "DataSetReader.h"

ImageFolderReader::ImageFolderReader(std::string path, std::string calibFile)
{
    this->path = path;
    this->calibFile = calibFile;

    getdir (path, files);

    loadTimestamps();

    printf("ImageFolderReader: got %d images and %d timestamps and %d exposures.!\n", (int)getNumImages(), (int)timestamps.size(), (int)exposures.size());
}

ImageFolderReader::~ImageFolderReader()
{

}

int ImageFolderReader::getNumImages()
{
	return files.size();
}

std::string ImageFolderReader::getImageFilename(int index)
{
    return files[index];
}

void ImageFolderReader::loadTimestamps()
{
    std::ifstream tr;
    std::string timesFile = path.substr(0,path.find_last_of('/')) + "/times.txt";
    tr.open(timesFile.c_str());
    while(!tr.eof() && tr.good())
    {
        std::string line;
        char buf[1000];
        tr.getline(buf, 1000);

        int id;
        double stamp;
        float exposure = 0;

        if(3 == sscanf(buf, "%d %lf %f", &id, &stamp, &exposure))
        {
            timestamps.push_back(stamp);
            exposures.push_back(exposure);
        }

        else if(1 == sscanf(buf, "%lf", &stamp))
        {
            timestamps.push_back(stamp);
            exposures.push_back(exposure);
        }

    }
    tr.close();

    // check if exposures are correct, (possibly skip)
    bool exposuresGood = ((int)exposures.size()==(int)getNumImages()) ;
    for(int i=0;i<(int)exposures.size();i++)
    {
        if(exposures[i] == 0)
        {
            // fix!
            float sum=0,num=0;
            if(i>0 && exposures[i-1] > 0) {sum += exposures[i-1]; num++;}
            if(i+1<(int)exposures.size() && exposures[i+1] > 0) {sum += exposures[i+1]; num++;}

            if(num>0)
                exposures[i] = sum/num;
        }

        if(exposures[i] == 0) exposuresGood=false;
    }


    if((int)getNumImages() != (int)timestamps.size())
    {
        printf("set timestamps and exposures to zero!\n");
        exposures.clear();
        timestamps.clear();
    }

    if((int)getNumImages() != (int)exposures.size() || !exposuresGood)
    {
        printf("set EXPOSURES to zero!\n");
        exposures.clear();
    }
}

double ImageFolderReader::getTimestamp(int index)
{
    if(timestamps.size()==0) return index*0.1f;

	if(index >= (int)timestamps.size()) return 0;

	if(index < 0) return 0;

	return timestamps[index];
}

std::vector<Matrix> ImageFolderReader::getGroundTruth()
{
    std::vector<Matrix> matrix_result;

        // show ground truth
    std::string gtPath = "/home/ljb2208/development/odometry/poses/00.txt";
    std::ifstream ReadFile(gtPath.c_str());
    std::string temp;
    std::string delim (" ");
    std::vector<std::string> results;
    Matrix gtCam = Matrix::eye(4);    

    while(std::getline(ReadFile, temp))
    {
        split(temp, delim, results);
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 4; j++)
            {
                gtCam.val[i][j] = atof(results[4*i + j].c_str());
            }
        gtCam.val[3][0] = 0;
        gtCam.val[3][1] = 0;
        gtCam.val[3][2] = 0;
        gtCam.val[3][3] = 1;

        results.clear();
        matrix_result.push_back(gtCam);        
    }
    ReadFile.close();
    return matrix_result;
}