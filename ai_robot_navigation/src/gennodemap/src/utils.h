#ifndef EXAMPLES_UTILS_H_
#define EXAMPLES_UTILS_H_

#include <fstream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

template <typename PointT, typename ContainerT>
void readPoints(const std::string& filename, ContainerT& points)
{
  std::ifstream in(filename.c_str());
  std::string line;
  boost::char_separator<char> sep(" ");
  PointT pt;
  float minx=3000, miny=3000,maxx=-3000,maxy=-3000, minz=3000, maxz=-3000;
  // read point cloud from "freiburg format"
  while (!in.eof())
  {
    std::getline(in, line);
    in.peek();

    boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

    if (tokens.size() != 3) continue;
    pt.x = boost::lexical_cast<float>(tokens[2]);
    pt.y = - boost::lexical_cast<float>(tokens[0]);
    pt.z = - boost::lexical_cast<float>(tokens[1]);
    pt.r = 0;
    pt.g = 255;
    pt.b = 0;

    points.push_back(pt);
    if(pt.x < minx){
        minx = pt.x;
    }
    if(pt.x > maxx){
        maxx = pt.x;
    }
    if(pt.y < miny){
        miny = pt.y;
    }
    if(pt.y > maxy){
        maxy = pt.y;
    }
    if(pt.z < minz){
        minz = pt.z;
    }
    if(pt.z > maxz){
        maxz = pt.z;
    }
  }

  std::cout << "x range (" << minx << "," << maxx << ")" << std::endl
            << "y range (" << miny << "," << maxy << ")" << std::endl
            << "z range (" << minz << "," << maxz << ")" << std::endl;
  in.close();
}

template <typename PointT, typename ContainerT>
void readPoints2(const std::string& filename, const std::string& filename2, ContainerT& points)
{
  std::ifstream in(filename.c_str());
  std::ifstream in2(filename2.c_str());
  std::string line, line2;
  boost::char_separator<char> sep(" ");
  PointT pt;
  float minx=3000, miny=3000,maxx=-3000,maxy=-3000, minz=3000, maxz=-3000;
  // read point cloud from "freiburg format"
  while (!in.eof())
  {
    std::getline(in, line);
    in.peek();
    std::getline(in2, line2);
    in2.peek();

    boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

    if (tokens.size() != 6) continue;
    pt.x = boost::lexical_cast<float>(tokens[3]);
    pt.y = boost::lexical_cast<float>(tokens[4]);
    pt.z = boost::lexical_cast<float>(tokens[5]);

    boost::tokenizer<boost::char_separator<char> > tokenizer2(line2, sep);
    std::vector<std::string> tokens2(tokenizer2.begin(), tokenizer2.end());
    if(tokens2.size()!=1) continue;
    //color by type
//    pt.r = (boost::lexical_cast<float>(tokens2[0]))/20*255;
//    pt.g = (20-boost::lexical_cast<float>(tokens2[0]))/20*255;
//    pt.b = (boost::lexical_cast<float>(tokens2[0]))/20*255;
    //color by z
    pt.r = (pt.z+5) * 30;
    pt.g = pt.r;
    pt.b = pt.r;

    if(pt.z > -1.3 && pt.z < -0.7)
    points.push_back(pt);

    if(pt.x < minx){
        minx = pt.x;
    }
    if(pt.x > maxx){
        maxx = pt.x;
    }
    if(pt.y < miny){
        miny = pt.y;
    }
    if(pt.y > maxy){
        maxy = pt.y;
    }
    if(pt.z < minz){
        minz = pt.z;
    }
    if(pt.z > maxz){
        maxz = pt.z;
    }
  }
  std::cout << "x range (" << minx << "," << maxx << ")" << std::endl
            << "y range (" << miny << "," << maxy << ")" << std::endl
            << "z range (" << minz << "," << maxz << ")" << std::endl;

  in.close();
}
#endif /* EXAMPLES_UTILS_H_ */
