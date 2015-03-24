
#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <utility>
#include <math.h>
#include <numeric>
#include <sstream>

#include <lama_common/point.h>
#include <lama_common/polygon.h>
#include <pm_fourier/fourier.h>
#include "utils.h"

#define intro if (id==-1) { cerr << myId << " : " << __FUNCTION__ << "\n"; return; } else if (id!=myId) return;

using namespace std;
using lama_common::Point2;

static void loadPolygon(const char *file, vector<Point2> &polygon) {
  ifstream ifs(file);
  polygon.clear();
  double x,y;
  while(ifs) {
    if (ifs >> x >> y) {
      polygon.push_back(Point2(x,y));
    }
  }
  ifs.close();
}

void getDissimilarity_fourier(int argc, char **argv, const int id, const int myId) {
  intro;
  if (argc < 3) {
    cerr << "usage: " << argv[0] << " <polygonFile1> <polygonFile2> <samples>\n";
    cerr << "numOfSamples .. for resampling polygons";
    cerr << "polygonFile* .. one point per line\n";
    exit(0);
  }

  const char *mapFile1 = argv[1];
  const char *mapFile2 = argv[2];
  //nst int numOfSamples = atoi(argv[3]);

  struct rusage t1,t2;
  getTime(&t1);
  vector<Point2> polygon1;
  vector<Point2> polygon2;
  loadPolygon(mapFile1,polygon1);
  loadPolygon(mapFile2,polygon2);

  double delta1;
  const int numOfSamples = 200;
  const int fftSize = 20;

  vector<Point2> rpol1(lama_common::resamplePolygon(polygon1,numOfSamples,delta1));
  vector<Point2> rpol2(lama_common::resamplePolygon(polygon2,numOfSamples,delta1));

  const double sim = getDissimilarityFourier(rpol1, rpol2,fftSize);
  getTime(&t2);
  getTime(t1,t2);
  rpol1.clear(); rpol2.clear(); polygon1.clear(); polygon2.clear();

  cout << sim << " " << getTime(t1,t2) << "\n";
}

void getDissimilarity_fourier_from_filelist(int argc, char **argv, const int id, const int myId) {
  intro;
  if (argc < 4) {
    cerr << "usage: " << argv[0] << " <fileList> <fftSize> <prefix>\n";
    cerr << "fileList  list of file pairs, each per pair line, these pair will be loaded and processed\n";
    exit(0);
  }
  const char *filelist = argv[1];
  const int fftSize = atoi(argv[2]);
  const char *prefix = argv[3];

  vector<std::pair<string,string> > files;
  ifstream ifs(filelist);
  while(ifs) {
    string a,b;
    if (ifs >> a >> b) {
      files.push_back(pair<string,string>(a,b));
    }
  }
  cerr << "Loaded " << files.size() << " file pairs from " << filelist << "\n";

  char name[2000];
  sprintf(name,"%s.fullresults",prefix);
  ofstream ofs(name);

  const int numOfSamples = 200;
  //    const int fftSize = 50;

  for(int i=0;i<(int)files.size();i++) {
    struct rusage t1,t2;
    getTime(&t1);
    vector<Point2> polygon1;
    vector<Point2> polygon2;
    loadPolygon(files[i].first.c_str(),polygon1);
    loadPolygon(files[i].second.c_str(),polygon2);

    double delta1;

    vector<Point2> rpol1(resamplePolygon(polygon1,numOfSamples,delta1));
    vector<Point2> rpol2(resamplePolygon(polygon2,numOfSamples,delta1));

    const double sim = getDissimilarityFourier(rpol1, rpol2,fftSize);
    getTime(&t2);
    getTime(t1,t2);
    rpol1.clear(); rpol2.clear(); polygon1.clear(); polygon2.clear();

    string f1a(files[i].first);
    string f2a(files[i].second);

    string f1( f1a.substr( f1a.rfind('/')+1)); 
    string f2( f2a.substr( f2a.rfind('/')+1)); 

    ofs << f1 << " " << f2 << " " << sim << " " << getTime(t1,t2) << "\n";
    ofs << f2 << " " << f1 << " " << sim << " " << getTime(t1,t2) << "\n";

    if (! (i % 1000)) {
      cerr <<  "Finished " << (100.0*i/files.size()) << "% \r";
    }
  }
  ofs.close();
}


int main(int argc, char **argv) {

  int k = -1;
  if (argc < 2) {
    cerr << "choose one of following:\n";
  } else {
    k = atoi(argv[1]);
    argc--;
    argv++;
  }
  getDissimilarity_fourier(argc,argv,k,1);
  getDissimilarity_fourier_from_filelist(argc,argv,k,2);
}

#undef intro


