
#ifndef _TAMAMQTEXMANAGER_
#define _TAMAMQTEXMANAGER_

#include "MyPolygon.h"
#include "TamaMQLib.h"
#include <Eigen/Dense>


transform3 translate(Vector3 v);
transform3 flipX();

class MQTexManager
{
public:
  MQTexManager() : dlg(NULL)
  {
  }
  std::vector<int> index;
  std::vector<CacheTexInfo> tex;
  DrawPolygonOptions *dlg;
  
  bool LoadDPMV(const char *path)
  {
    FILE *fp = fopen(path, "rb");
    if(fp==NULL)return false;
    bool bRet = _LoadDPMV(fp);
    fclose(fp);
    return bRet;
  }
  
  bool _LoadDPMV(FILE *fp)
  {
    if(fgetc(fp)!='D')return false;
    if(fgetc(fp)!='P')return false;
    if(fgetc(fp)!='M')return false;
    if(fgetc(fp)!='V')return false;
    if(fgetc(fp)!=0x0)return false;
    
    unsigned int numMaterials = 0;
    if(fread(&numMaterials, 4, 1, fp)!=1)return false;
    
    //for(int i=0;i<numMaterials;i++)index.push_back(-2/*-1*/);
    index.resize(numMaterials, -2/*-1*/);
    
    for(int i=0;i<numMaterials;i++)
    {
      unsigned int w = 0;
      if(fread(&w, 4, 1, fp)!=1)return false;
      unsigned int h = 0;
      if(fread(&h, 4, 1, fp)!=1)return false;
      
      unsigned int numMatId = 0;
      if(fread(&numMatId, 4, 1, fp)!=1)return false;
      
      std::vector<int> matIDs;
      for(int n=0;n<numMatId;n++)
      {
        int id = 0;
        if(fread(&id, sizeof(int), 1, fp)!=1)return false;
        matIDs.push_back(id);
      }
      
      tex.push_back(CacheTexInfo());
      int newidx = tex.size()-1;
      CacheTexInfo &ci = tex[newidx];
      bool ret = ci.o.read(fp);
      if(!ret)return false;
      
      ci.w = w;
      ci.h = h;
      __UpdateMinMaxXY(ci);
      for(int n=0;n<numMatId;n++)
      {
        index[matIDs[n]] = newidx;
      }
    }
    return true;
  }
  
  
  bool LoadDPMR(const char *path)
  {
    FILE *fp = fopen(path, "rb");
    if(fp==NULL)return false;
    bool bRet = _LoadDPMR(fp);
    fclose(fp);
    return bRet;
  }
  
  bool _LoadDPMR(FILE *fp)
  {
    if(fgetc(fp)!='D')return false;
    if(fgetc(fp)!='P')return false;
    if(fgetc(fp)!='M')return false;
    if(fgetc(fp)!='R')return false;
    if(fgetc(fp)!=0x0)return false;
    
    unsigned int numMaterials = 0;
    if(fread(&numMaterials, 4, 1, fp)!=1)return false;
    
    //for(int i=0;i<numMaterials;i++)index.push_back(-2/*-1*/);
    index.resize(numMaterials, -2/*-1*/);
    
    for(int i=0;i<numMaterials;i++)
    {
      unsigned int numMatId = 0;
      if(fread(&numMatId, 4, 1, fp)!=1)return false;
      
      std::vector<int> matIDs;
      for(int n=0;n<numMatId;n++)
      {
        int id = 0;
        if(fread(&id, sizeof(int), 1, fp)!=1)return false;
        matIDs.push_back(id);
      }
      
      //filename
      unsigned int sizeFilename = 0;
      if(fread(&sizeFilename, 4, 1, fp)!=1)return false;
      
      char *filename = (char *)malloc(sizeFilename+17);
      if(filename==NULL)return false;
      
      if(fread(filename, sizeFilename, 1, fp)!=1)return false;
      memset(filename+sizeFilename, NULL, 16); //NULL
      
      cv::Mat mat = cv::imread(filename);
      if(mat.empty())return -2;
      
      free(filename);
      filename=NULL;
      
      tex.push_back(CacheTexInfo());
      int newidx = tex.size()-1;
      CacheTexInfo &ci = tex[newidx];
      
      ci.matRaster = mat;
      
      ci.w = mat.cols;
      ci.h = mat.rows;
      __UpdateMinMaxXY(ci);
      for(int n=0;n<numMatId;n++)
      {
        index[matIDs[n]] = newidx;
      }
    }
    return true;
  }
private:
  /*
  int __FindTextureIndex(const char *fullpath)
  {
    for(int i=0;i<tex.size();i++)
    {
      if(tex[i].fullpath == fullpath)return i;
    }
    return -1;
  }
  
  MQObject FindMQObjectByName(char *name, int *_oi = NULL)
  {
    return _FindMQObjectByName(doc, name, _oi);
  }
  
  MQObject FindChildMQObjectByName(char *name, MQObject oParent, int *_oi = NULL)
  {
    return _FindChildMQObjectByName(doc, name, oParent, _oi);
  }
  
  void MakeCacheName(const char *fullpath, int w, int h, char *cachename)
  {
    strcpy(cachename, fullpath);
    PathStripPathA(cachename);
    PathRemoveExtensionA(cachename);
    int len = strlen(cachename);
    char *pNull = cachename+len;
    sprintf(pNull, "_%d_%d", w, h);
  }
  
  bool MakeTexCache(const char *cachename, MQObject mqoCacheRoot, CacheTexInfo &t, int matIdx)
  {
    if(mqoCacheRoot!=NULL && dlg!=NULL)
    {
      bool bFacegen = dlg->check_facegen->GetChecked();
      bool bOptimize = dlg->check_optimize->GetChecked();
      bool bThresholdMennuki = dlg->check_thresholdMennuki->GetChecked();
      
      cv::Mat &mat = t.matRaster;
      CRaster2Vector conv(*dlg);
      if(conv.Raster2Vector(mat)==false)return false;
      
      MQObject o = MQ_CreateObject();
      o->SetName(cachename);
      
      double wf = mat.cols, hf = mat.rows;
      
      cv::Mat uvMat(2, 3, CV_64F);
      uvMat.at<double>(0,0) = 1.0/wf;
      uvMat.at<double>(1,1) = 1.0/hf;
      transform3 invMat =  flipX()*translate(Vector3(0.0, -hf, 0.0));*/
  //    if(bFacegen)conv.OutputToMetaseq_LineTri(o, uvMat, bThresholdMennuki, &invMat, true/*bInvertFace*/, matIdx);
 /*     else        conv.OutputToMetaseq_LineOnly(o);
      
      if(bOptimize)o->OptimizeVertex(0.0f, NULL);
      doc->InsertObject(o, mqoCacheRoot);
      o->SetDepth(1);
      t.o = o;
      
      conv.reset();
      return true;
    }
    return false;
  }
  
  bool checkMQOParent(MQObject o, const char *parentname)
  {
    if(o==NULL)return false;
    MQObject oParent = doc->GetParentObject(o);
    if(oParent==NULL)return false;
    
    char tmp[_MAX_PATH*2+2];
    oParent->GetName(tmp, _MAX_PATH*2);
    if(strcmp(tmp, parentname)==0)return true;
    return false;
  }*/
  
  void __UpdateMinMaxXY(CacheTexInfo &info)
  {
    MyObject mqoTex = &(info.o);
    double minx, miny, maxx, maxy;
    minx = miny = FLT_MAX;
    maxx = maxy = FLT_MIN;
    
    int numTexF = mqoTex->GetFaceCount();
    for(int fiTex=0;fiTex<numTexF;fiTex++)
    {
      int numTexFV = mqoTex->GetFacePointCount(fiTex);
      if(numTexFV==0)continue;
      std::vector<MyPoint> ptsTex(numTexFV);
      std::vector<MyCoordinate> coordTex(numTexFV);
      GetPoint(mqoTex, fiTex, numTexFV, ptsTex);
      
      for(int i=0;i<numTexFV;i++)
      {
        MyPoint &p = ptsTex[i];
        float x2 = p.x;
        float y2 = p.y;
        //float x2 = -(p.x)+info.w;
        //float y2 = -(p.y)+info.h;
        if(x2 > maxx)maxx = x2;
        if(y2 > maxy)maxy = y2;
        if(x2 < minx)minx = x2;
        if(y2 < miny)miny = y2;
      }
    }
    info.minx = minx;
    info.miny = miny;
    info.maxx = maxx;
    info.maxy = maxy;
  }
  /*
  int __LoadMQTexture(int matIdx)
  {
    MQMaterial mqmat = doc->GetMaterial(matIdx);
    if(mqmat==NULL)return -2;
    
    char texPath[_MAX_PATH*2 + 2];
    mqmat->GetTextureName(texPath, _MAX_PATH*2);
    char texFullPath[_MAX_PATH*2 + 2];
    if(!doc->FindMappingFile(texFullPath, texPath, MQMAPPING_TEXTURE))return -2;
    
    int texidx = __FindTextureIndex(texFullPath);
    if(texidx!=-1)return texidx;
    
    cv::Mat mat = cv::imread(texFullPath);
    if(mat.empty())return -2;
    
    tex.push_back(CacheTexInfo());
    int newidx = tex.size()-1;
    if(newidx<0 || newidx>=tex.size())return -2;
    
    CacheTexInfo &t = tex[newidx];
    t.fullpath = texFullPath;
    t.matRaster = mat;
    if(bVectorize) {
      MQObject mqoTex = NULL;
      MQObject mqoCacheRoot = FindMQObjectByName("MQDrawPolygonTexCache");
      if(mqoCacheRoot==NULL)
      {
        mqoCacheRoot = MQ_CreateObject();
        if(mqoCacheRoot==NULL)return -3;
        mqoCacheRoot->SetName("MQDrawPolygonTexCache");
        doc->AddObject(mqoCacheRoot);
      }
      char cachename[_MAX_PATH*2 + 2];
      int w = mat.cols, h = mat.rows;
      MakeCacheName(texFullPath, w, h, cachename);
      if(mqoCacheRoot!=NULL)
      {
        mqoTex = FindChildMQObjectByName(cachename, mqoCacheRoot);
        t.o = mqoTex;
      }
      if(mqoTex==NULL)
      {
        MakeTexCache(cachename, mqoCacheRoot, t, matIdx);
        mqoTex = t.o;
      }
      TriangulateObj(doc, mqoTex);
      t.w = w;
      t.h = h;
      __UpdateMinMaxXY(t);
      
    }
    return newidx;
  }*/
  
  int _FindIndex(int matIdx)
  {
    if(matIdx==-1 || matIdx>=index.size())return -1;
    int texStat = index[matIdx];
    if(texStat==-2)return -1;
    if(texStat==-1)
    {
      return -1;
      /*
      int ret = __LoadMQTexture(matIdx);
      index[matIdx]=ret;
      if(ret<0)return -1;
      */
    }
    return index[matIdx];
  }
public:
  
  CacheTexInfo* GetTexture(MyObject o, int fi)
  {
    int mqMatIdx = o->GetFaceMaterial(fi);
    int idx = _FindIndex(mqMatIdx);
    if(idx < 0 || idx >= tex.size())return NULL;
    return &(tex[idx]);
  }
};

#endif //_TAMAMQTEXMANAGER_
