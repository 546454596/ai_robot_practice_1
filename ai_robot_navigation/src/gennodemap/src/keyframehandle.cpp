#include "keyframehandle.h"

//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
const unibn::OctreeParams& keyfOctParams = unibn::OctreeParams(2, false, 0.1f);//16 for lab,32 for sim,12 for lidar
const unibn::OctreeParams& mapOctParams = unibn::OctreeParams(128, false, 0.05f);

template<typename PointT>
double calDist2(PointT a, PointT b){
    return ((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

PathNode::PathNode(float _x, float _y, float _z, int _id):
    x(_x),y(_y),z(_z),id_self(_id), id_fromWhere(-1), isDealed(false),
    distTilNow(-1), distToEnd(-1), distTotal(-1), to_delete(0)
{}

PathNode::~PathNode()
{}

void PathNode::setxyzid(float _x, float _y, float _z, int _id)
{
    x = _x;
    y = _y;
    z = _z;
    id_self = _id;
}

void PathNode::addLink(int _id_link, float _dist)
{
    id_link.push_back(_id_link);
    dist_link.push_back(_dist);
}

void PathNode::removeLink(int _id_link)
{
    for(int i=0; i<id_link.size(); ++i){
        if(_id_link == id_link[i]){
            id_link.erase(id_link.begin()+i);
            dist_link.erase(dist_link.begin()+i);
            break;
        }
    }
}

void PathNode::replaceLink(int _id_orilink, int _id_afterlink)
{
    bool replaced = false;
    for(int i=0; i<id_link.size(); ++i){
        if(_id_afterlink == id_link[i]){
            if(replaced){
                id_link.erase(id_link.begin()+i);
            }
            break;
        }
        if(_id_orilink == id_link[i]){// || _id_afterlink== id_link[i]
            id_link[i] = _id_afterlink;
            replaced = true;
        }
    }
}

float PathNode::isLinkded(int _id_link)
{
    for(int i=0; i<id_link.size(); ++i){
        if(_id_link == id_link[i]){
            return dist_link[i];
        }
    }
    return -1.0;
}

KeyFrameHandler::KeyFrameHandler(const string &mappointfile, const string &keyframefile):
    v1(0),v2(0),isview(true),topPercent(0.25),minKeyFdist(5.0),mappointSparse(0.35), maxlinkdist(6.0),
    near_zero_dealt(false),
    mapPC(new pcl::PointCloud<pcl::PointXYZRGB>),
    mapPCorigin(new pcl::PointCloud<pcl::PointXYZRGB>),
    mapPCfinal(new pcl::PointCloud<pcl::PointXYZRGB>),
    keyfPC(new pcl::PointCloud<pcl::PointXYZRGB>),
    denkeyfPC(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    viewer.reset(new pcl::visualization::PCLVisualizer ("generate graph"));
    readParams("../src/params_for_key_frame.yaml");
    readRemoveMapP();
    readMapPt(mappointfile);
    readKeyFrame(keyframefile);
    initPclViewer();
}

KeyFrameHandler::KeyFrameHandler(const string &paramfile):
    v1(0),v2(0),isview(false),topPercent(0.25),minKeyFdist(5.0),mappointSparse(0.35), maxlinkdist(6.0),
    near_zero_dealt(false),
    mapPC(new pcl::PointCloud<pcl::PointXYZRGB>),
    mapPCorigin(new pcl::PointCloud<pcl::PointXYZRGB>),
    mapPCfinal(new pcl::PointCloud<pcl::PointXYZRGB>),
    keyfPC(new pcl::PointCloud<pcl::PointXYZRGB>),
    denkeyfPC(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    readParams(paramfile);
}

KeyFrameHandler::~KeyFrameHandler()
{}

void KeyFrameHandler::clearall()
{
    mapPCfinal->clear();
    mapPC->clear();
    mapPCorigin->clear();
    keyfPC->clear();
    denkeyfPC->clear();
    denkeyfLine.clear();
    pNode.clear();
    to_delete_vec.clear();
    near_zero_dealt = false;
}

void KeyFrameHandler::reset(string savepath)
{
    clearall();
    string mappointfile = savepath+"Input/MapPointsPos.txt"
         , keyframefile = savepath+"Input/trajectory.txt"
         , bcfile = savepath+"Topomap/backup_pos.txt";
    readMapPt(mappointfile);
    readKeyFrame(keyframefile);
    readBackupPos(bcfile);
}

void KeyFrameHandler::reset(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _mapPC, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _keyfPC)
{
    clearall();
    pcl::copyPointCloud(*_mapPC, *mapPC);
    pcl::copyPointCloud(*_mapPC, *mapPCfinal);
    pcl::copyPointCloud(*_keyfPC, *keyfPC);
}


void KeyFrameHandler::dealKeyFrame()
{
    keyfOct.initialize(keyfPC->points, keyfOctParams);
    mapOct.initialize(mapPC->points, mapOctParams);
    killErrMapP();
    findDenseKeyFrame();
    lineDenseKeyFrame();
    removeRedundantLine();
    // removeRedundantNode();
}

void KeyFrameHandler::display()
{
    //display origin map point and keyframe
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchmapPC(mapPCorigin);
    viewer->addPointCloud<pcl::PointXYZRGB> (mapPCorigin, pchmapPC, "origin map point cloud", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "origin map point cloud", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchkeyfPC(keyfPC);
    viewer->addPointCloud<pcl::PointXYZRGB> (keyfPC, pchkeyfPC, "origin keyframe cloud", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "origin keyframe cloud", v1);
    viewer->addCoordinateSystem (1.0, "origin cloud", v1);
    //add line of keyframe
    string li("line00000");
    for(int i=0; i<keyfPC->size()-1; ++i)
    {
        setstring(li, i);
        viewer->addLine(keyfPC->points[i], keyfPC->points[i+1], 0, 255, 0, li, v1);
    }

    //display dealed map point and keyframe pos
    viewer->addPointCloud<pcl::PointXYZRGB> (mapPCfinal, pchmapPC, "dealed map point cloud", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "dealed map point cloud", v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pchdenkeyfPC(denkeyfPC);
    viewer->addPointCloud<pcl::PointXYZRGB> (denkeyfPC, pchdenkeyfPC, "dealed keyframe cloud", v2);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "dealed keyframe cloud", v2);
    viewer->addCoordinateSystem (1.0, "dealed cloud", v2);
    //add line of dense keyframe
    string lii("dline00000");
    /*for(int i=0; i<denkeyfLine.size(); ++i)
    {
        setstring(lii, i);
        viewer->addLine(denkeyfPC->points[denkeyfLine[i][0]], denkeyfPC->points[denkeyfLine[i][1]], 0, 255, 0, lii, v2);
    }*/

    pcl::PointXYZRGB po1, po2;
    int ii=0;
    for(int i=0; i<pNode.size(); ++i){
        po1.x = pNode[i].x;
        po1.y = pNode[i].y;
        po1.z = pNode[i].z;
        // cout<<"po1: "<<po1.x<<", "<<po1.y<<", "<<po1.z<<endl
        //     <<"den: "<<denkeyfPC->points[i].x<<", "<<denkeyfPC->points[i].y<<", "<<denkeyfPC->points[i].z<<endl;
        for(int j=0; j<pNode[i].id_link.size(); ++j){
//            if(!pNode[i].isDealed) continue;
            if(pNode[i].id_link[j] > i){
                setstring(lii, ii);
                ++ii;
                po2.x = pNode[pNode[i].id_link[j]].x;
                po2.y = pNode[pNode[i].id_link[j]].y;
                po2.z = pNode[pNode[i].id_link[j]].z;
                // cout<<"edge: "<<i<<" "<<pNode[i].id_link[j]<<endl
                //     <<"po2: "<<po2.x<<", "<<po2.y<<", "<<po2.z<<endl
                //     <<"den: "<<denkeyfPC->points[pNode[i].id_link[j]].x<<", "<<denkeyfPC->points[pNode[i].id_link[j]].y<<", "<<denkeyfPC->points[pNode[i].id_link[j]].z<<endl;

                if(pNode[i].isDealed){
                    viewer->addLine(po1, po2, 0, 0, 255, lii, v2);
                }
                else{
                    viewer->addLine(po1, po2, 0, 255, 0, lii, v2);
                }
            }
        }
    }


    while(isview)
    {
        viewer->spinOnce (100);
    }
}

void KeyFrameHandler::saveResultOnlyFinal(string savepath)
{
    cout << "saving node and edge..." << endl;

    ofstream of(savepath+"node.txt")
            ,of1(savepath+"edge.txt");
    for(int i=0; i<pNode.size(); ++i){
        bool isDealed = pNode[i].isDealed;
        if(isDealed){
            of << pNode[i].x << " "
               << pNode[i].y << " "
               << pNode[i].z << endl;
            for(int j=0; j<pNode[i].id_link.size(); ++j){
                if(pNode[i].id_link[j] > i){
                    of1 << (i - to_delete_vec[i]) << " "
                        << (pNode[i].id_link[j] - to_delete_vec[pNode[i].id_link[j]]) << endl;
                }
            }
        }
    }
    of.close();
    of1.close();

    ofstream of2(savepath+"MapPointsPos.txt");
    for(int i=0; i<mapPCfinal->size(); ++i)
    {
        of2 << mapPCfinal->points[i].x << " "
            << mapPCfinal->points[i].y << " "
            << mapPCfinal->points[i].z << endl;
    }
    of2.close();

    cout << "save done!" << endl;
}

void KeyFrameHandler::saveResultPN(string savepath)
{
    cout << "saving dense keyframe position and and link relation..." << endl;

    ofstream of(savepath+"denKeyfPos.txt")
             , of1(savepath+"denKeyfPosRelation.txt")
            , of3(savepath+"node.txt")
            , of4(savepath+"edge.txt");
    for(int i=0; i<pNode.size(); ++i){
        of << pNode[i].x << " "
           << pNode[i].y << " "
           << pNode[i].z << endl;
        bool isDealed = pNode[i].isDealed;
        if(isDealed){
            of3 << pNode[i].x << " "
               << pNode[i].y << " "
               << pNode[i].z << endl;
        }
        for(int j=0; j<pNode[i].id_link.size(); ++j){
            if(pNode[i].id_link[j] > i){
                of1 << i << " " << pNode[i].id_link[j] << endl;
                if(isDealed){
                    of4 << (i - to_delete_vec[i]) << " "
                        << (pNode[i].id_link[j] - to_delete_vec[pNode[i].id_link[j]]) << endl;
                }
            }
        }
    }
    of.close();
    of1.close();
    of3.close();
    of4.close();

    ofstream of2(savepath+"MapPointsPos.txt");
    for(int i=0; i<mapPCfinal->size(); ++i)
    {
        of2 << mapPCfinal->points[i].x << " "
            << mapPCfinal->points[i].y << " "
            << mapPCfinal->points[i].z << endl;
    }
    of2.close();

    cout << "save done!" << endl;
}

void KeyFrameHandler::saveResultPN()
{
    cout << "saving dense keyframe position and and link relation..." << endl;

    ofstream of("../Result/denKeyfPos1.txt")
             , of1("../Result/denKeyfPosRelation1.txt")
            , of3("../Result/node.txt")
            , of4("../Result/edge.txt");
    for(int i=0; i<pNode.size(); ++i){
        of << pNode[i].x << " "
           << pNode[i].y << " "
           << pNode[i].z << endl;
        bool isDealed = pNode[i].isDealed;
        if(isDealed){
            of3 << pNode[i].x << " "
               << pNode[i].y << " "
               << pNode[i].z << endl;
        }
        for(int j=0; j<pNode[i].id_link.size(); ++j){
            if(pNode[i].id_link[j] > i){
                of1 << i << " " << pNode[i].id_link[j] << endl;
                if(isDealed){
                    of4 << (i - to_delete_vec[i]) << " "
                        << (pNode[i].id_link[j] - to_delete_vec[pNode[i].id_link[j]]) << endl;
                }
            }
        }
    }
    of.close();
    of1.close();
    of3.close();
    of4.close();

    ofstream of2("../Result/MapPointsPos.txt");
    for(int i=0; i<mapPCfinal->size(); ++i)
    {
        of2 << mapPCfinal->points[i].x << " "
            << mapPCfinal->points[i].y << " "
            << mapPCfinal->points[i].z << endl;
    }
    of2.close();

    cout << "save done!" << endl;
}

void KeyFrameHandler::saveResult()
{
    cout << "saving dense keyframe position and and link relation..." << endl;

    ofstream of("../Result/denKeyfPos.txt");

    for(int i=0; i<denkeyfPC->size(); ++i)
    {
        of << denkeyfPC->points[i].x << " "
           << denkeyfPC->points[i].y << " "
           << denkeyfPC->points[i].z << endl;
    }
    of.close();

    ofstream of1("../Result/denKeyfPosRelation.txt");
    for(int i=0; i<denkeyfLine.size(); ++i)
    {
        of1 << denkeyfLine[i][0] << " "
            << denkeyfLine[i][1] << endl;
    }
    of1.close();

    ofstream of2("../Result/MapPointsPos.txt");
    for(int i=0; i<mapPCfinal->size(); ++i)
    {
        of2 << mapPCfinal->points[i].x << " "
            << mapPCfinal->points[i].y << " "
            << mapPCfinal->points[i].z << endl;
    }
    of2.close();

    cout << "save done!" << endl;
}

void KeyFrameHandler::saveResult(string savepath)
{
    cout << "saving dense keyframe position and and link relation..." << endl;

    ofstream of(savepath+"denKeyfPos.txt");
    ofstream of1(savepath+"denKeyfPosRelation.txt");
/*
    for(int i=0; i<denkeyfPC->size(); ++i)
    {
        of << denkeyfPC->points[i].x << " "
           << denkeyfPC->points[i].y << " "
           << denkeyfPC->points[i].z << endl;
    }
    of.close();

    for(int i=0; i<denkeyfLine.size(); ++i)
    {
        of1 << denkeyfLine[i][0] << " "
            << denkeyfLine[i][1] << endl;
    }
    of1.close();
*/
    for(int i=0; i<pNode.size(); ++i){
        of << pNode[i].x << " "
           << pNode[i].y << " "
           << pNode[i].z << endl;
        for(int j=0; j<pNode[i].id_link.size(); ++j){
            if(pNode[i].id_link[j] > i){
                of1 << i << " " << pNode[i].id_link[j] << endl;
            }
        }
    }
    of.close();
    of1.close();

    ofstream of2(savepath+"MapPointsPos.txt");
    for(int i=0; i<mapPCfinal->size(); ++i)
    {
        of2 << mapPCfinal->points[i].x << " "
            << mapPCfinal->points[i].y << " "
            << mapPCfinal->points[i].z << endl;
    }
    of2.close();

    cout << "save done!" << endl;
}

void KeyFrameHandler::readMapPt(const string &mpfile)
{
    std::ifstream in(mpfile.c_str());
    if(!in.is_open())
    {
        std::cout << "cannot read "<< mpfile.c_str() <<std::endl;
        return;
    }
    std::string line;
    boost::char_separator<char> sep(" ");
    pcl::PointXYZRGB pt;
    float minx=3000, miny=3000,maxx=-3000,maxy=-3000, minz=3000, maxz=-3000;
    bool skipthispoint = false;
    // read point cloud from "freiburg format"
    while (!in.eof())
    {
        //cout << "read mappoint" << endl;
        std::getline(in, line);
        in.peek();
        //cout << "read mappoint1" << endl;
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        //cout << "read mappoint2" << tokens.size() << endl;
        if (tokens.size() != 3) continue;
        if(DEALMODE == 0){
        //orbslam
        pt.x = boost::lexical_cast<float>(tokens[2]);
        pt.y = - boost::lexical_cast<float>(tokens[0]);
        pt.z = - boost::lexical_cast<float>(tokens[1]);
        }
        else if(DEALMODE == 1){
        //cartographer
        pt.x = boost::lexical_cast<float>(tokens[0]);
        pt.y = boost::lexical_cast<float>(tokens[1]);
        pt.z = boost::lexical_cast<float>(tokens[2]);
        }
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;

        mapPCorigin->push_back(pt);

        for(int i=0; i<removeMappointArea.size(); ++i)
        {
            if(pt.x > removeMappointArea[i][0] && pt.x < removeMappointArea[i][1] &&
               pt.y > removeMappointArea[i][2] && pt.y < removeMappointArea[i][3] &&
               pt.z > removeMappointArea[i][4] && pt.z < removeMappointArea[i][5]){
                skipthispoint = true;
                //break;
            }
        }

        if(skipthispoint)
        {
            skipthispoint = false;
            continue;
        }

        mapPC->push_back(pt);
        mapPCfinal->push_back(pt);
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

    std::cout << "read map point pos----->" << std::endl
              << "  x range (" << minx << "," << maxx << ")" << std::endl
              << "  y range (" << miny << "," << maxy << ")" << std::endl
              << "  z range (" << minz << "," << maxz << ")" << std::endl
              << "  total " << mapPC->size() << " map points." << endl;
    in.close();
}

void KeyFrameHandler::readBackupPos(const string &bcpfile)
{
    std::ifstream in(bcpfile.c_str());
    if(!in.is_open())
    {
        std::cout << "cannot read "<< bcpfile.c_str() <<std::endl;
        return;
    }
    std::string line;
    boost::char_separator<char> sep(" ");
    PathNode pt(0,0,0,0);
    backupNode.clear();

    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();

        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

        if (tokens.size() != 3) continue;

        pt.x = boost::lexical_cast<float>(tokens[0]);
        pt.y = boost::lexical_cast<float>(tokens[1]);
        pt.z = boost::lexical_cast<float>(tokens[2]);

        backupNode.push_back(pt);
    }

    cout << "read back up nodes" << endl
         << " total " << backupNode.size() << endl;
}

void KeyFrameHandler::readKeyFrame(const string &kffile)
{
    std::ifstream in(kffile.c_str());
    if(!in.is_open())
    {
        std::cout << "cannot read "<< kffile.c_str() <<std::endl;
        return;
    }
    std::string line;
    boost::char_separator<char> sep(" ");
    pcl::PointXYZRGB pt, pttmp;
    pt.r = 0;
    pt.g = 0;
    pt.b = 255;
    pttmp.r = 0;
    pttmp.g = 0;
    pttmp.b = 255;
    float minx=3000, miny=3000,maxx=-3000,maxy=-3000, minz=3000, maxz=-3000;
    float lastx = 0, lasty = 0, lastz = 0;
    // read point cloud from "freiburg format"
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();

        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

        if (tokens.size() != 8) continue;
        if(DEALMODE == 0){
        //orbslam
        pt.x = boost::lexical_cast<float>(tokens[3]);
        pt.y = - boost::lexical_cast<float>(tokens[1]);
        pt.z = - boost::lexical_cast<float>(tokens[2]);
        }
        else if(DEALMODE == 1){
        //cartographer
        pt.x = boost::lexical_cast<float>(tokens[1]);
        pt.y = boost::lexical_cast<float>(tokens[2]);
        pt.z = boost::lexical_cast<float>(tokens[3]);
        }
        keyfPC->push_back(pt);
        if(keyfPC->size() > 1){
            float dist = sqrt(pow(lastx-pt.x, 2) + pow(lastz-pt.z, 2) + pow(lastz-pt.z, 2));
            if(dist > 1.5 && dist < 10){
                //insert more keyf points
                int insert_num = dist / minKeyFdist * 3;
                // cout<<"insert keyf point "<<insert_num<<" before point "<<keyfPC->size()<<endl;
                for(int i=1; i<insert_num; ++i)
                {
                    pttmp.x = pt.x + (lastx-pt.x) / insert_num * i;
                    pttmp.y = pt.y + (lasty-pt.y) / insert_num * i;
                    pttmp.z = pt.z + (lastz-pt.z) / insert_num * i;
                    // cout<<" p "<<pttmp.x<<" "<<pttmp.y<<" "<<pttmp.z<<endl;
                    keyfPC->push_back(pttmp);
                }
            }
        }
        lastx = pt.x;
        lasty = pt.y;
        lastz = pt.z;
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

    std::cout << "read keyframe pos----->" << std::endl
              << "  x range (" << minx << "," << maxx << ")" << std::endl
              << "  y range (" << miny << "," << maxy << ")" << std::endl
              << "  z range (" << minz << "," << maxz << ")" << std::endl
              << "  total " << keyfPC->size() << " keyframe pos." << endl;
    in.close();
}

void KeyFrameHandler::readRemoveMapP()
{
    std::ifstream in("../src/removeMapP.txt");
    if(!in.is_open())
    {
        std::cout << "cannot read ../src/removeMapP.txt" <<std::endl;
    }
    std::string line;
    boost::char_separator<char> sep(" ");
    vector<float> tmp;
    tmp.resize(6);
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();

        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

        if (tokens.size() != 6) continue;
//        tmp.push_back( boost::lexical_cast<float>(tokens[0]));cout<<tmp[0]<<endl;
//        tmp.push_back( boost::lexical_cast<float>(tokens[1]));cout<<tmp[1]<<endl;
//        tmp.push_back( boost::lexical_cast<float>(tokens[2]));cout<<tmp[2]<<endl;
//        tmp.push_back( boost::lexical_cast<float>(tokens[3]));cout<<tmp[3]<<endl;
//        tmp.push_back( boost::lexical_cast<float>(tokens[4]));cout<<tmp[4]<<endl;
//        tmp.push_back( boost::lexical_cast<float>(tokens[5]));cout<<tmp[5]<<endl;
        tmp[0] = boost::lexical_cast<float>(tokens[0]);
        tmp[1] = boost::lexical_cast<float>(tokens[1]);
        tmp[2] = boost::lexical_cast<float>(tokens[2]);
        tmp[3] = boost::lexical_cast<float>(tokens[3]);
        tmp[4] = boost::lexical_cast<float>(tokens[4]);
        tmp[5] = boost::lexical_cast<float>(tokens[5]);

        removeMappointArea.push_back(tmp);
    }
    //cout<<"here:"<<removeMappointArea[0].size()<<endl;

    std::cout << "read remove area----->" <<removeMappointArea.size()<<" area."<<std::endl;
    for(int i=0; i<removeMappointArea.size(); ++i)
    {
        std::cout << "x(" << removeMappointArea[i][0] << "," << removeMappointArea[i][1] <<"),"
                  << "y(" << removeMappointArea[i][2] << "," << removeMappointArea[i][3] <<"),"
                  << "z(" << removeMappointArea[i][4] << "," << removeMappointArea[i][5] <<"),"<<std::endl;
    }
    in.close();
}

void KeyFrameHandler::readParams(const string &paramfile)
{
    cv::FileStorage fs(paramfile, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        cout << "can not find parameter file ../src/params_for_key_frame.yaml" << endl
             << "default parameters are used!" << endl;
        return;
    }
    fs["DEALMODE"] >> DEALMODE;
    fs["topPercent"] >> topPercent;
    fs["minKeyFdist"] >> minKeyFdist;
    fs["mappointSparse"] >> mappointSparse;
    fs["maxlinkdist"] >> maxlinkdist;
    cout << "read parameters----->" << endl
         << "  DEALMODE:" << DEALMODE << endl
         << "  topPercent:" << topPercent << endl
         << "  minKeyFdist:" << minKeyFdist << endl
         << "  mappointSparse:" << mappointSparse <<endl
         << "  maxlinkdist:" << maxlinkdist <<endl;
}

void KeyFrameHandler::initPclViewer()
{
    viewer->initCameraParameters();
    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);
    viewer->setBackgroundColor (255, 255, 255, v1);
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->setBackgroundColor (255, 255, 255, v2);
}

void KeyFrameHandler::setstring(string &str, int k)
{
    int t = 0, i = 0, size = str.size();
    char b = '0';
    while(k>0)
    {
        t = k%10;
        k = k/10;
        b = b + t;
        str.insert(str.end()-i-1, b);
        str.erase(size-i,1);
        b = b - t;
        ++i;
    }
}

void KeyFrameHandler::findDenseKeyFrame()
{
    //vector<float*> rank_dense_Keyf;
    //if(DEALMODE == 0){
    keyfOct.rankDenseGrid(denkeyfPC->points, topPercent, minKeyFdist);
    //}
    //else if(DEALMODE == 1){
    //    denkeyfPC = keyfPC;
    //}
    cout << "dense keyframe pos number:" << denkeyfPC->size() << endl;
}

void KeyFrameHandler::lineDenseKeyFrame()
{
    vector<int> a(2);
    for(int i=0; i<denkeyfPC->size()-1; ++i)
    {
        for(int j=i+1; j<denkeyfPC->size(); ++j)
        {
            double _dist2 = pow(denkeyfPC->points[i].x-denkeyfPC->points[j].x, 2) + pow(denkeyfPC->points[i].y-denkeyfPC->points[j].y, 2);
            //here is a param
            if( (_dist2 < maxlinkdist) && (!mapOctfinal.isBlock<unibn::L2Distance<pcl::PointXYZRGB> >(denkeyfPC->points[i], denkeyfPC->points[j], mappointSparse)) )
            {
                a[0] = i;
                a[1] = j;
                denkeyfLine.push_back(a);
            }
        }
    }
    cout << "dense keyframe line number:" << denkeyfLine.size() << endl;
}

void KeyFrameHandler::removeRedundantNode()
{
//    return;
    cout << "removing redundant nodes..." << endl;
    //judge crossing
    vector<int> near_zero_node_vec;
    for(int i=0; i<pNode.size(); ++i){
        //skip some nodes on straight path, isdealed is set true if this node is left
        // special case: when there is only one loop, there will be no crossing so make the zero node as crossing
        if(pNode[i].id_link.size() != 2 || pNode[i].isDealed){
            if((fabs(pNode[i].x) + fabs(pNode[i].y) + fabs(pNode[i].z)) < 1){
                near_zero_dealt = true;
            }
            pNode[i].isDealed = true;
            for(int j=0; j<pNode[i].id_link.size(); ++j){
                int id_nextnode = pNode[i].id_link[j], id_lastcrossing = i;
                // walk along to next crossing
                goToCheckNextCrossing(id_lastcrossing, id_nextnode);
            }
        }
        else if(!near_zero_dealt && (fabs(pNode[i].x) + fabs(pNode[i].y) + fabs(pNode[i].z)) < 1){
            near_zero_node_vec.push_back(i);
        }
    }
    if(!near_zero_dealt && near_zero_node_vec.size() > 0){
        int i = near_zero_node_vec[0];
        pNode[i].isDealed = true;
        for(int j=0; j<pNode[i].id_link.size(); ++j){
            int id_nextnode = pNode[i].id_link[j], id_lastcrossing = i;
            // walk along to next crossing
            goToCheckNextCrossing(id_lastcrossing, id_nextnode);
        }
    }
    to_delete_vec.resize(pNode.size());
    int to_delete = 0;
    for(int i=0; i<pNode.size(); ++i){
        if(!pNode[i].isDealed){
            ++to_delete;
        }
        to_delete_vec[i] = to_delete;
    }
    cout << "remove total " << to_delete << " nodes." << endl;
}


void KeyFrameHandler::goToCheckNextCrossing(int _id_lastcrossing, int _id_nextnode)
{
    // the next node is crossing at one step
    if(pNode[_id_nextnode].isDealed || (pNode[_id_nextnode].id_link.size() != 2)){return;}

    int id_lastcrossing=_id_lastcrossing, id_nextnode=_id_nextnode
        , id_lastnode=_id_lastcrossing, id_nextnextnode=-1;
    //delete the link between crossing and its neibor
//    pNode[_id_lastcrossing].removeLink(_id_nextnode);
    // pNode[_id_nextnode].removeLink(_id_lastcrossing);
    //do not stop until to a dead end or crossing
    do{
        //get the other node of next node
        if(pNode[id_nextnode].id_link[0] == id_lastnode){
            id_nextnextnode = pNode[id_nextnode].id_link[1];
        }
        else{
            id_nextnextnode = pNode[id_nextnode].id_link[0];
        }
        if(pNode[id_nextnode].isDealed == true || mapOctfinal.isBlock<unibn::L2Distance<pcl::PointXYZRGB> >(denkeyfPC->points[id_nextnextnode], denkeyfPC->points[id_lastcrossing], 1.0*mappointSparse))
        {
            //make this node a crossing
            pNode[id_nextnode].isDealed = true;
            //remove link with old neibor, new dist will not be calculated here
            // pNode[id_lastnode].removeLink(id_nextnode);
            pNode[id_nextnode].replaceLink(id_lastnode, id_lastcrossing);
            pNode[id_lastcrossing].replaceLink(_id_nextnode, id_nextnode);
            goToCheckNextCrossing(id_nextnode, id_nextnextnode);
            return;
        }
        else{
            //go on to next node
            id_lastnode = id_nextnode;
            id_nextnode = id_nextnextnode;
        }
    }while(pNode[id_nextnode].id_link.size() == 2);

    //go to a crossing, link it to last crossing,
    //remove link between crossing and noncrossing node
    // pNode[id_lastnode].removeLink(id_nextnode);
    pNode[id_nextnode].replaceLink(id_lastnode, id_lastcrossing);
    pNode[id_nextnode].isDealed = true;
    pNode[id_lastcrossing].replaceLink(_id_nextnode, id_nextnode);
}

//there is a python for this function
void KeyFrameHandler::removeRedundantLine()
{
    cout << "removing redundantlink..." << endl;
    pNode.clear();
    PathNode pn(0,0,0,0);
    //make nodes
    Eigen::VectorXd min_norms = Eigen::VectorXd::Constant(backupNode.size(), 10.0);
    Eigen::VectorXi id_near_backup_node = Eigen::VectorXi::Constant(backupNode.size(), -1);
    for(int i=0; i<denkeyfPC->size(); ++i){
        pn.setxyzid(denkeyfPC->points[i].x, denkeyfPC->points[i].y, denkeyfPC->points[i].z, i);
        pNode.push_back(pn);
        // find nodes near back up position
        for(int bc_i=0; bc_i<backupNode.size(); ++bc_i)
        {
            double dist_center = norm(pNode[i], backupNode[bc_i]);
            if(dist_center < min_norms(bc_i)){
                min_norms(bc_i) = dist_center;
                id_near_backup_node(bc_i) = i;
            }
        }
    }
    // these nodes are set as important nodes
    for(int bc_i=0; bc_i<backupNode.size(); ++bc_i)
    {
        pNode[id_near_backup_node[bc_i]].isDealed = true;
    }
    //make links
    for(int i=0; i<denkeyfLine.size(); ++i){
        int m = denkeyfLine[i][0];
        int n = denkeyfLine[i][1];
        double dist = calDist2(denkeyfPC->points[m], denkeyfPC->points[n]);
        pNode[m].addLink(n, dist);
        pNode[n].addLink(m, dist);
    }
    //vector< vector<int> >
    //remove links
    int cnt = 0;
    for(int inode_id=0; inode_id<pNode.size(); ++inode_id){
        if(pNode[inode_id].id_link.size() <= 2){
            continue;
        }
        else{
            //see if loop between three nodes(i,j,k)
            for(int j=0; j<pNode[inode_id].id_link.size(); ++j){
                int jnode_id = pNode[inode_id].id_link[j];
                for(int k=j+1; k<pNode[inode_id].id_link.size(); ++k){
                    int knode_id = pNode[inode_id].id_link[k];
                    // check if two nodes have a link
                    float dist_jk = pNode[knode_id].isLinkded(jnode_id);
                    if( dist_jk > 0 ){
                        float dist_ij = pNode[inode_id].dist_link[j], dist_ik = pNode[inode_id].dist_link[k];
                        // remove longest link
                        if(dist_ij > dist_ik){
                            if(dist_ij > dist_jk){
                                pNode[inode_id].removeLink(jnode_id);
                                pNode[jnode_id].removeLink(inode_id);
                            }
                            else{
                                pNode[knode_id].removeLink(jnode_id);
                                pNode[jnode_id].removeLink(knode_id);
                            }
                        }
                        else{
                            if(dist_ik > dist_jk){
                                pNode[inode_id].removeLink(knode_id);
                                pNode[knode_id].removeLink(inode_id);
                            }
                            else{
                                pNode[knode_id].removeLink(jnode_id);
                                pNode[jnode_id].removeLink(knode_id);
                            }
                        }
                        ++cnt;
                    }
                }
            }
        }
    }
    cout << "remove total " << cnt << " edges." << endl;
}

void KeyFrameHandler::killErrMapP()
{
    std::vector<uint32_t> results, allPointToKill;
    //find all points along keyframe path
    for(int i=1; i < keyfPC->size(); ++i)
    {
        if(fabs(keyfPC->points[i].x - keyfPC->points[i-1].x) > 2 || fabs(keyfPC->points[i].y - keyfPC->points[i-1].y) > 2){
            continue;
        }
        //mapOct.radiusNeighbors<unibn::L2Distance<pcl::PointXYZRGB> >(keyfPC->points[i], 0.25f, results);
        findPointAlongTwoPos(results, keyfPC->points[i], keyfPC->points[i-1]);
        allPointToKill.insert(allPointToKill.end(), results.begin(), results.end());
    }
    //stamp points
    for(int i=0; i<allPointToKill.size(); ++i)
    {
        mapPCfinal->points[allPointToKill[i]].x = 0;
        mapPCfinal->points[allPointToKill[i]].y = 0;
        mapPCfinal->points[allPointToKill[i]].z = 0;
    }
    //remove points
    for(int i=0; i<mapPCfinal->size(); ++i)
    {
        if(fabs(mapPCfinal->points[i].x) + fabs(mapPCfinal->points[i].y) + fabs(mapPCfinal->points[i].z) < 0.01)
        {
            mapPCfinal->points.erase(mapPCfinal->points.begin()+i);
            --i;
        }
    }

    mapOctfinal.initialize(mapPCfinal->points, mapOctParams);
    cout<<"after removing map points: "<<mapPCfinal->size()<<endl;
}

void KeyFrameHandler::findPointAlongTwoPos(std::vector<uint32_t>& results, pcl::PointXYZRGB p1, pcl::PointXYZRGB p2){
    if((pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) + pow(p1.z-p2.z, 2)) > 1.2*mappointSparse){
        pcl::PointXYZRGB mid;
        mid.x = (p1.x + p2.x) / 2;
        mid.y = (p1.y + p2.y) / 2;
        mid.z = (p1.z + p2.z) / 2;
        findPointAlongTwoPos(results, mid, p1);
        findPointAlongTwoPos(results, mid, p2);
    }
    else{
        std::vector<uint32_t> _res;
        mapOct.radiusNeighbors<unibn::L2Distance<pcl::PointXYZRGB> >(p1, 1.2*mappointSparse, _res);
        results.insert(results.end(), _res.begin(), _res.end());
        _res.clear();
        mapOct.radiusNeighbors<unibn::L2Distance<pcl::PointXYZRGB> >(p2, 1.2*mappointSparse, _res);
        results.insert(results.end(), _res.begin(), _res.end());
    }
}

double KeyFrameHandler::norm(PathNode p1, PathNode p2)
{
    return sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) + pow(p1.z-p2.z, 2));
}
