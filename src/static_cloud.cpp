//
// Created by mrs on 25/06/2021.
//

#include "static_cloud.h"
#include <pcl/io/pcd_io.h>
#include <filesystem>

pc_XYZ_t::Ptr load_cloud(const std::string &filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::ifstream fs;
    std::cout << std::filesystem::current_path() << std::endl;
    fs.open(filename.c_str(), std::ios::binary);
    if (!fs.is_open() || fs.fail()) {
        PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(), strerror(errno));
        fs.close();
        return nullptr;
    }

    while (!fs.eof()) {
        std::string line;
        std::getline(fs, line);
        // Ignore empty lines
        if (line.empty())
            continue;

        // Tokenize the line
        boost::trim(line);
        /* const std::vector<std::string> st = split(line, ' '); */
        std::vector<std::string> st;
        boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

        if (st.size() != 3) {
            PCL_WARN("Read line with a wrong number of elements: %d (expected 3). The line: '%s'.", (int) st.size(),
                     line.c_str());
            continue;
        }

        cloud->push_back(pcl::PointXYZ(
                float(atof(st[0].c_str())),
                float(atof(st[1].c_str())),
                float(atof(st[2].c_str()))
        ));

        if (cloud->size() % 100000 == 0)
            PCL_INFO("Loaded %lu points so far.\r\n", cloud->size());
    }
    fs.close();

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}
