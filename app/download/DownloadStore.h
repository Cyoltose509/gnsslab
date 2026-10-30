#pragma once

#include <string>
#include <vector>

#include "ProductMeta.h"


struct DownloadEntry {
    std::string type;
    int         year = 0;
    int         doy  = 0;
    std::string datacenter;
    std::string file;
    std::string path;
    long long   size = 0;
};



class DownloadStore {
public:

    static std::string root();


    static std::string dirFor(ProductType t, int year, int doy);


    static std::string localPath(ProductType t, int year, int doy, const std::string &filename);


    static std::vector<std::string> findLocal(ProductType t, int year, int doy);


    static std::string localNameFor(const std::string &remoteName);


    static std::string findLocalFile(ProductType t, int year, int doy, const std::string &remoteName);


    static bool removeFile(const std::string &path);


    static void record(const DownloadEntry &e);



    static std::string decompressIfNeeded(const std::string &localPath);


    static std::vector<DownloadEntry> readManifest();
    static void writeManifest(const std::vector<DownloadEntry> &all);
};
