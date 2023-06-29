#include "keyframehandle.h"

int main(int argc, char** argv)
{
    // if (argc < 3)
    // {
    //   std::cerr << "filename of point cloud missing." << std::endl;
    //   return -1;
    // }
    std::string mappointfilename = "../Input/MapPointsPos.txt";//argv[1];
    std::string keyframefilename = "../Input/trajectory.txt";//argv[2];
    int64_t begin, end;
    KeyFrameHandler kfh(mappointfilename, keyframefilename);
    begin = clock();
    kfh.dealKeyFrame();
    end = clock();

    // kfh.saveResult();
    kfh.saveResultPN();

    double search_time = ((double)(end - begin) / CLOCKS_PER_SEC);
    cout << "total deal time: " << search_time << " seconds." <<endl;
    cout << "ctrl+c to exit."<<endl;
    kfh.display();
    return 1;
}
