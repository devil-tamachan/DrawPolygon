
#if defined _WIN32 || defined __CYGWIN__
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <float.h>

#if defined _WIN32 || defined __CYGWIN__
#include <minmax.h>
#endif
//#include "MQPlugin.h"
//#include "MQWidget.h"
#include "My3DStruct.h"


#if defined _WIN32 || defined __CYGWIN__
//MQSDKのバージョンによってGetObject(atlgdi.hなど)が使えなくなる
//http://www.metaseq.net/bbs/metaseq/bbs.php?lang=jp&res=7044
#if MQPLUGIN_VERSION >= 0x0459
#ifndef GetObject
inline int GetObject(HGDIOBJ p1, int p2, LPVOID p3)
{
#ifdef UNICODE
  return GetObjectW(p1,p2,p3);
#else
  return GetObjectA(p1,p2,p3);
#endif
}
#endif
#endif
#include <atlbase.h>
#include <atlapp.h>
#include <atlmisc.h>
#endif


#include <iostream>
#include <list>

#define CGAL_INTERSECTION_VERSION 2


#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Optimal_transportation_reconstruction_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Barycentric_coordinates_2/Triangle_coordinates_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "TamaMQLib.h"
#include "CacheTexInfo.h"

//#define MYOUTPUTDEBUG

#ifdef MYOUTPUTDEBUG
void MyOutputDebugStringA(const char *s) { OutputDebugStringA(s); }
#else
void MyOutputDebugStringA(const char *s) { }
#endif

//typedef CGAL::Simple_cartesian<double> K;
//typedef CGAL::Simple_cartesian<double> K2;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K; //CGAL::intersectionでAccess Violationがおきる https://stackoverflow.com/questions/32951886/wrong-inexact-intersection-between-3d-triangles-cgal   追記: intersectionへの入力ポリゴンが不正だった可能性大。回転方向とarea0に気をつける必要あり。is_simple、 include/CGAL/Boolean_set_operations_2/Gps_polygon_validation.hなど参照
typedef CGAL::Exact_predicates_exact_constructions_kernel K2; //Optimal_transportation_reconstruction_2とんでもなく遅い。無限ループなのか判断できないほど遅い
//typedef CGAL::Exact_predicates_inexact_constructions_kernel K2;

typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point3;
typedef K::Point_2 Point2;
typedef K::Triangle_2 Triangle2;
typedef K::Iso_rectangle_2 Iso_rectangle2;
typedef K::Triangle_3 Triangle3;
typedef K::Direction_3 Direction;
typedef K::Vector_2 Vector2;
typedef K::Vector_3 Vector3;
typedef K::Segment_2 Segment2;
typedef CGAL::Polygon_2<K> Polygon2;

typedef CGAL::Aff_transformation_3<K> transform3; //Point3以外(Vector3など)は移動成分無視されるので注意

typedef std::list<Triangle3>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;

typedef std::pair<Point2, FT> PointMassPair;
typedef std::vector<PointMassPair> PointMassList;
typedef CGAL::First_of_pair_property_map <PointMassPair> Point_property_map;
typedef CGAL::Second_of_pair_property_map <PointMassPair> Mass_property_map;
typedef CGAL::Optimal_transportation_reconstruction_2<K, Point_property_map, Mass_property_map> Otr_2;
typedef CGAL::Optimal_transportation_reconstruction_2<K> Otr;


typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, CGAL::Default, Itag> CDT;


typedef CGAL::Barycentric_coordinates::Triangle_coordinates_2<K> Triangle_coordinates;

//typedef K2::Triangle_2 Triangle2_K2;
typedef CGAL::Polygon_2<K2> Polygon2_K2;
typedef CGAL::Polygon_with_holes_2<K2> Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2> Pwh_list_2;
typedef K2::Point_2 Point2_K2;




  cv::Point2f point2ToCvPoint2f(Point2 &p)
  {
    double x = CGAL::to_double(p.x());
    double y = CGAL::to_double(p.y());
    return cv::Point2f(x, y);
  }
  cv::Point point2ToCvPoint(Point2 &p)
  {
    double x = CGAL::to_double(p.x());
    double y = CGAL::to_double(p.y());
    return cv::Point(x, y);
  }
  cv::Point point2_K2ToCvPoint(Point2_K2 &p)
  {
    double x = CGAL::to_double(p.x());
    double y = CGAL::to_double(p.y());
    return cv::Point(x, y);
  }

#ifdef MYOUTPUTDEBUG
void OutputDebugStringVector3(Vector3 v)
{
  char buf[1025];
  sprintf(buf, "Vector3: %f, %f, %f\n", float(CGAL::to_double(v.x())), float(CGAL::to_double(v.y())), float(CGAL::to_double(v.z())));
  OutputDebugStringA(buf);
}
void OutputDebugStringVector2(Vector2 v)
{
  char buf[1025];
  sprintf(buf, "Vector2: %f, %f\n", float(CGAL::to_double(v.x())), float(CGAL::to_double(v.y())));
  OutputDebugStringA(buf);
}
void OutputDebugCVSize(cv::Size s)
{
  char buf[1025];
  sprintf(buf, "OutputDebugCVSize = (%d, %d)\n", s.width, s.height);
  OutputDebugStringA(buf);
}
#else
inline void OutputDebugStringVector3(Vector3 v) {}
inline void OutputDebugStringVector2(Vector2 v) {}
inline void OutputDebugCVSize(cv::Size s) {}
#endif


#include "DrawPolygonOptions.h"
#include "Raster2Vector.h"
#include "MQTexManager.h"


#if defined _WIN32 || defined __CYGWIN__
bool GetClipboardBitmap(cv::Mat &mat)
{
  if(!::OpenClipboard(NULL))return false;
  HBITMAP hBitmap = (HBITMAP)::GetClipboardData(CF_BITMAP);
  BITMAP bmp;
  if(hBitmap==NULL || !GetObject(hBitmap, sizeof(BITMAP), &bmp))
  {
    ::CloseClipboard();
    return false;
  }
  BITMAPINFO bi = {0};
  BITMAPINFOHEADER *bh = &(bi.bmiHeader);
  bh->biSize = sizeof(BITMAPINFOHEADER);
  bh->biWidth = bmp.bmWidth;
  bh->biHeight = -bmp.bmHeight;
  bh->biPlanes = 1;
  bh->biBitCount = 32;
  bh->biCompression = BI_RGB;
  
  CDC dc;
  dc.CreateCompatibleDC();
  HBITMAP hBitmapOld = dc.SelectBitmap(hBitmap);
  mat.create(bmp.bmHeight, bmp.bmWidth, CV_8UC4);
  GetDIBits(dc, hBitmap, 0, bmp.bmHeight, mat.data, &bi, DIB_RGB_COLORS);
  cv::flip(mat, mat, 0);
  
  dc.SelectBitmap(hBitmapOld);
  ::CloseClipboard();
  
  return true;
}

bool ConvertFromClipboard(DrawPolygonOptions &opt, const char *outPath)
{
  /*
  int edgeproc = dlg.combo_edgeproc->GetCurrentIndex();
  //int vectorconv = dlg.combo_vectorconv->GetCurrentIndex();
  //double threshold = dlg.slider_threshold->GetPosition();
  //int np = dlg.spin_np->GetPosition();
  bool bFacegen = dlg.check_facegen->GetChecked();
  //int thresholdMennuki = dlg.slider_thresholdMennuki->GetPosition() * 255.0;
  bool bOptimize = dlg.check_optimize->GetChecked();
  bool bThresholdMennuki = dlg.check_thresholdMennuki->GetChecked();
  */
  
  cv::Mat mat;
  if(GetClipboardBitmap(mat)==false)return false;
  
  CRaster2Vector conv(opt);
  
  if(conv.Raster2Vector(mat)==false)return false;

  //printf("type = %d, depth = %d, channels = %d\n", dstEdge.type(), dstEdge.depth(), dstEdge.channels());

  _MyObject o;

  double wf = mat.cols, hf = mat.rows;
  
  cv::Mat uvMat(2, 3, CV_64F, cv::Scalar(0));
  uvMat.at<double>(0,0) = 1.0/wf;
  uvMat.at<double>(1,1) = -(1.0/hf);
  if(opt.bFacegen)conv.OutputToMetaseq_LineTri(&o, uvMat, opt.bThresholdMennuki);
  else        conv.OutputToMetaseq_LineOnly(&o);

  
  conv.reset();
  
  FILE *fpOut = fopen(outPath, "wb");
  if(fpOut==NULL)return false;
  
  o.write(fpOut);
  fclose(fpOut);
  
  //const char* source_window = "Source";
  //cv::namedWindow( source_window, cv::WINDOW_NORMAL );
  //imshow( source_window, dstEdge );
  //cv::waitKey(0);
  
  return true;
}

#endif

transform3 rotateX(double angle)
{
#ifdef MYOUTPUTDEBUG
  char buf[1025];
  double degree = angle*180.0/M_PI;
  sprintf(buf, "rotateX: %f rad, %f degree\n", angle,degree);
  OutputDebugStringA(buf);
#endif

  double cosa = cos(angle);
  double sina = sin(angle);
  return transform3(
      1,   0,    0,
      0,cosa,-sina,
      0,sina, cosa
  );
}
transform3 rotateY(double angle)
{
#ifdef MYOUTPUTDEBUG
  char buf[1025];
  double degree = angle*180.0/M_PI;
  sprintf(buf, "rotateY: %f rad, %f degree\n", angle,degree);
  OutputDebugStringA(buf);
#endif

  double cosa = cos(angle);
  double sina = sin(angle);
  return transform3(
      cosa, 0, sina,
      0,    1,    0,
      -sina,0, cosa
  );
}
transform3 rotateZ(double angle)
{
#ifdef MYOUTPUTDEBUG
  char buf[1025];
  double degree = angle*180.0/M_PI;
  sprintf(buf, "rotateZ: %f rad, %f degree\n", angle,degree);
  OutputDebugStringA(buf);
#endif

  double cosa = cos(angle);
  double sina = sin(angle);
  return transform3(
      cosa,-sina,0,
      sina, cosa,0,
      0,       0,1
  );
}
transform3 translate(Vector3 v)
{
  return transform3(
      1, 0, 0, v.x(),
      0, 1, 0, v.y(),
      0, 0, 1, v.z()
  );
}
transform3 flipX()
{
  return transform3(
      1.0,  0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0,  0.0, 1.0, 0.0
  );
}


Vector3 normalize(Vector3 v)
{
  double l = sqrt(CGAL::to_double(v.squared_length()));
  return (l==0.0) ? Vector3(0.0, 0.0, 0.0) : v/l;
}
Vector2 normalize(Vector2 v)
{
  double l = sqrt(CGAL::to_double(v.squared_length()));
  return (l==0.0) ? Vector2(0.0, 0.0) : v/l;
}


//transform()の注意: transform3の移動成分は、Pointのみ作用するので注意。CGALの仕様。Point以外のVectorなどをかけても移動しない
//前提条件: 同じ大きさのポリゴンであること。拡縮成分は計算されない
bool Tri3Dto2D(Vector3 *tri, Vector3 *newtri, transform3 &invMat)
{
  //Vector3 v2(tri[1].x-tri[0].x, tri[1].y-tri[0].y, tri[1].z-tri[0].z);
  //Vector3 v3(tri[2].x-tri[0].x, tri[2].y-tri[0].y, tri[2].z-tri[0].z);
  Vector3 v2 = tri[1] - tri[0];
  Vector3 v3 = tri[2] - tri[0];
  
  transform3 t1i(CGAL::TRANSLATION, tri[0]);
  //p3
  OutputDebugStringVector3(tri[0]);
  OutputDebugStringVector3(tri[1]);
  OutputDebugStringVector3(tri[2]);
  OutputDebugStringVector3(v2);
  OutputDebugStringVector3(v3);
  Vector2 vn = normalize(Vector2(v3.x(), v3.y()));
  OutputDebugStringVector2(vn);
  double angle1 = acos(CGAL::to_double(vn.x()));
  if(v3.y()>0.0)angle1 = -angle1;
  transform3 r1 = rotateZ(angle1);
  transform3 r1i = rotateZ(-angle1);
  v2 = r1.transform(v2);
  v3 = r1.transform(v3);
  Vector2 vn2 = normalize(Vector2(v3.x(), v3.z()));
  double angle2 = acos(CGAL::to_double(vn2.x()));
  if(v3.z()>0.0)angle2 = -angle2;
  transform3 r2 = rotateY(-angle2);
  transform3 r2i = rotateY(angle2);
  v2 = r2.transform(v2);
  v3 = r2.transform(v3);
  //p2
  Vector2 vn3 = normalize(Vector2(v2.y(), v2.z()));
  double angle3 = acos(CGAL::to_double(vn3.x()));
  if(v2.z()>0.0)angle3 = -angle3;
  transform3 r3 = rotateX(angle3);
  transform3 r3i = rotateX(-angle3);
  v2 = r3.transform(v2);
  v3 = r3.transform(v3);
  
  
  OutputDebugStringVector3(v2);
  OutputDebugStringVector3(v3);
  
  newtri[0] = Vector3(0.0,0.0,0.0);
  newtri[1] = v2;
  newtri[2] = v3;
  
  invMat = t1i * r1i * r2i * r3i;     // (r3 * (r2 *(r1 * (t1 * v)))) = v2, (t1i * (r1i * (r2i * (r3i * v2)))) = v, (t1i * r1i * r2i * r3i) * v2 = v; http://www.geisya.or.jp/~mwm48961/kou2/matrix3.html
  
  return true;
}

/*

*/
void CalcSizeAndShift(Vector3 *tri2d, int *w, int *h, double *shiftx, double *shifty, double padding = 1.0)
{
  double minx = DBL_MAX, maxx = DBL_MIN;
  double miny = DBL_MAX, maxy = DBL_MIN;
  for(int i=0;i<3;i++)
  {
    double x = CGAL::to_double(tri2d[i].x());
    double y = CGAL::to_double(tri2d[i].y());
    minx = MIN(x, minx);
    maxx = MAX(x, maxx);
    miny = MIN(y, miny);
    maxy = MAX(y, maxy);
  }
  
  int minx2 = (minx - padding);
  int miny2 = (miny - padding);
  int maxx2 = (maxx + 0.5 + padding);
  int maxy2 = (maxy + 0.5 + padding);
  *w = maxx2-minx2;
  *h = maxy2-miny2;
  *shiftx = minx - padding;
  *shifty = miny - padding;
  
#ifdef MYOUTPUTDEBUG
  char buf[1025];
  sprintf(buf, "CalcSizeAndShift: w=%d, h=%d, shiftx=%f, shifty=%f\n", *w, *h, *shiftx, *shifty);
  OutputDebugStringA(buf);
#endif
}

bool CalcCoord(Point2 &pt, Triangle_coordinates &triangle_coordinates, MyCoordinate *triuv, MyCoordinate &retUV)
{
  std::vector<FT> coord(3);
  triangle_coordinates(pt, std::inserter(coord, coord.end()));
  
  retUV.u = triuv[0].u * CGAL::to_double(coord[0]) + triuv[1].u * CGAL::to_double(coord[1]) + triuv[2].u * CGAL::to_double(coord[2]);
  retUV.v = triuv[0].v * CGAL::to_double(coord[0]) + triuv[1].v * CGAL::to_double(coord[1]) + triuv[2].v * CGAL::to_double(coord[2]);
  
  return (coord[0]>=0.0 && coord[1]>=0.0 && coord[2]>=0.0);
}

cv::Mat CalcRasterizeMatrix(Vector3 *tri2d, double shiftx, double shifty, MyCoordinate *triuv, cv::Size srcSize, cv::Mat *matUVCalc = NULL)
{
  cv::Point2f src[3], src2[3];
  cv::Point2f dst[3], dst2[3];
  
  for(int i=0;i<3;i++)
  {
    src[i] = cv::Point2f(triuv[i].u * srcSize.width, triuv[i].v * srcSize.height);
    dst[i] = cv::Point2f(CGAL::to_double(tri2d[i].x())-shiftx, CGAL::to_double(tri2d[i].y())-shifty);
    src2[i] = cv::Point2f(triuv[i].u, triuv[i].v);
    dst2[i] = cv::Point2f(CGAL::to_double(tri2d[i].x()), CGAL::to_double(tri2d[i].y()));
  }
  
#ifdef MYOUTPUTDEBUG
  char buf[1025];
  for(int i=0;i<3;i++)
  {
    sprintf(buf, "src[%d] = %f, %f\n", i, src[i].x, src[i].y);
    OutputDebugStringA(buf);
  }
  for(int i=0;i<3;i++)
  {
    sprintf(buf, "dst[%d] = %f, %f\n", i, dst[i].x, dst[i].y);
    OutputDebugStringA(buf);
  }
#endif
  if(matUVCalc!=NULL)
  {
    (*matUVCalc) = cv::getAffineTransform(dst2, src2);
  }
  
  return cv::getAffineTransform(src, dst);
}

bool GetTriangleBitmap(cv::Mat &matSrc, cv::Mat &matDst, Vector3 *tri2d, MyCoordinate *triuv, double *shiftx, double *shifty, double padding, cv::Mat &matUVCalc)
{
  int w, h;
  CalcSizeAndShift(tri2d, &w, &h, shiftx, shifty, padding);
  
  Triangle_coordinates triangle_coordinates(Point2(tri2d[0].x(), tri2d[0].y()), Point2(tri2d[1].x(), tri2d[1].y()), Point2(tri2d[2].x(), tri2d[2].y()));
  
  OutputDebugCVSize(matSrc.size());
  
  matDst.create(h, w, CV_8UC4);
  matDst = cv::Scalar(0xFF);
  cv::Mat matRaster = CalcRasterizeMatrix(tri2d, *shiftx, *shifty, triuv, matSrc.size(), &matUVCalc);
  warpAffine(matSrc, matDst, matRaster, matDst.size(), cv::INTER_LANCZOS4, cv::BORDER_WRAP);
  
  OutputDebugCVSize(matDst.size());
  /*
  unsigned char *p = matDst.data;
  for(int k=0;k<h;k++)
  {
    for(int i=0;i<w;i++)
    {
      Point2 pt(i + shiftx, k + shifty);
      MQCoordinate coord;
      bool bHit = CalcCoord(pt, triangle_coordinates, triuv, coord);
      for(int m=0;m<4;m++)
      {
        *p = 0xFF;
        p++;
      }
    }
  }
  */
  //cv::flip(matDst, matDst, 0);
  
  return true;
}




void WhiteOutsideTri(cv::Mat &mat, Point2 *tri2dShift)
{
  cv::Mat mask = cv::Mat(mat.rows, mat.cols, CV_8UC1);
  mask.setTo(cv::Scalar(0));
  cv::Point points[3];
  points[0] = point2ToCvPoint(tri2dShift[0]);
  points[1] = point2ToCvPoint(tri2dShift[1]);
  points[2] = point2ToCvPoint(tri2dShift[2]);
  cv::fillConvexPoly(mask, points, 3, cv::Scalar(255));
  cv::Mat maskNot = cv::Mat(mat.rows, mat.cols, CV_8UC1);
  cv::bitwise_not(mask, maskNot);
  mat.setTo(cv::Scalar(255,255,255,255), maskNot);
}


bool ConvertFromTextureSlow(DrawPolygonOptions &opt, MyObject oSrc, MQTexManager &texManager, const char *outPath)
{
  /*
  int edgeproc = dlg.combo_edgeproc->GetCurrentIndex();
  //int vectorconv = dlg.combo_vectorconv->GetCurrentIndex();
  //double threshold = dlg.slider_threshold->GetPosition();
  //int np = dlg.spin_np->GetPosition();
  bool bFacegen = dlg.check_facegen->GetChecked();
  //int thresholdMennuki = dlg.slider_thresholdMennuki->GetPosition() * 255.0;
  bool bOptimize = dlg.check_optimize->GetChecked();
  bool bThresholdMennuki = dlg.check_thresholdMennuki->GetChecked();
  */
  
  FILE *fpOut = fopen(outPath, "wb");
  if(fpOut==NULL)return false;
  
  //TriangulateSelected(doc);
  
  int vidx[3];
  MyCoordinate triuv[3];
  MyPoint pts[3];
  Vector3 tri[3];
  Vector3 tri2d[3];
  transform3 invMat;
  
  //int numMaterial = doc->GetMaterialCount();
  //std::vector<int> texLoadStatus(numMaterial, -1);
  //std::vector<std::pair<std::string, cv::Mat>> texLoaded;
  //MQTexManager texManager;
    
  /*
  MQObject oOut = MQ_CreateObject();
  char objname[151];
  doc->GetUnusedObjectName(objname, 150, "draw");
  oOut->SetName(objname);
  */
  _MyObject _oOut;
  MyObject oOut = &_oOut;
  
  CRaster2Vector conv(opt);
  
  int numV = oSrc->GetVertexCount();
  int numF = oSrc->GetFaceCount();
  for(int fi=0;fi<numF;fi++)
  {
    int numFV = oSrc->GetFacePointCount(fi);
    if(numFV!=3)continue;
    
    CacheTexInfo *pTexInfo = texManager.GetTexture(oSrc, fi);
    if(pTexInfo==NULL)continue;
    cv::Mat &texMat = pTexInfo->matRaster;
    
    GetPointAndCoord(oSrc, fi, 3, vidx, pts, triuv);
    MyPointToVector3(pts, tri, 3);
    Tri3Dto2D(tri, tri2d, invMat);
    cv::Mat mat, matUVCalc;
    double shiftx, shifty;
    double padding = 10.0;
    GetTriangleBitmap(texMat, mat, tri2d, triuv, &shiftx, &shifty, padding, matUVCalc);
    
    Point2 tri2dShift[3];
    for(int i=0;i<3;i++)
    {
      tri2dShift[i] = Point2(tri2d[i].x() - shiftx, tri2d[i].y() - shifty);
    }
    
    WhiteOutsideTri(mat, tri2dShift);
  MyOutputDebugStringA("-GetTriangleBitmap\n");
    
    
      /*
  const char* source_window = "Source";
  cv::namedWindow( source_window, cv::WINDOW_NORMAL );
  imshow( source_window, mat );
  cv::waitKey(0);
  */
  
    if(conv.Raster2Vector(mat)==false)continue;
    
    conv.AddPoints(tri2d, 3);
    
      //printf("type = %d, depth = %d, channels = %d\n", dstEdge.type(), dstEdge.depth(), dstEdge.channels());

      //float wf = dstEdge.cols, hf = dstEdge.rows;
      
  MyOutputDebugStringA("+OutputToMetaseq\n");
      
    int mqMatIdx = oSrc->GetFaceMaterial(fi);
    Point2 shift2d(shiftx, shifty);
    if(opt.bFacegen)conv.OutputToMetaseq_LineTri(oOut, matUVCalc, opt.bThresholdMennuki, &invMat, false, mqMatIdx, tri2d, &shift2d, padding);
    else        conv.OutputToMetaseq_LineOnly(oOut, &invMat, tri2d, &shift2d, padding);
  MyOutputDebugStringA("-OutputToMetaseq\n");
    
    conv.reset();
  }
  
  conv.reset();
  
  oOut->write(fpOut);
  fclose(fpOut);
  
  //const char* source_window = "Source";
  //cv::namedWindow( source_window, cv::WINDOW_NORMAL );
  //imshow( source_window, dstEdge );
  //cv::waitKey(0);
  MyOutputDebugStringA("End\n");
  
  return true;
}

bool ConvertFromTextureFast(DrawPolygonOptions &opt, MyObject oSrc, MQTexManager &texManager, const char *outPath)
{
  /*
  int edgeproc = dlg.combo_edgeproc->GetCurrentIndex();
  //int vectorconv = dlg.combo_vectorconv->GetCurrentIndex();
  //double threshold = dlg.slider_threshold->GetPosition();
  //int np = dlg.spin_np->GetPosition();
  bool bFacegen = dlg.check_facegen->GetChecked();
  //int thresholdMennuki = dlg.slider_thresholdMennuki->GetPosition() * 255.0;
  bool bOptimize = dlg.check_optimize->GetChecked();
  int modeZScale = dlg.combo_zscale->GetCurrentIndex();
  double zscale = dlg.dblspin_zscale->GetPosition();
  */
  
  FILE *fpOut = fopen(outPath, "wb");
  if(fpOut==NULL)return false;
  
  //TriangulateSelected(doc);
  
  int vidxTri[3];
  MyCoordinate triuv[3];
  MyPoint pts[3];
  Vector3 tri[3];
  Vector3 tri2d[3];
  transform3 invMat;
  
  //int numMaterial = doc->GetMaterialCount();
  //std::vector<int> texLoadStatus(numMaterial, -1);
  //std::vector<std::pair<std::string, cv::Mat>> texLoaded;
  //MQTexManager texManager;
    
    /*
  MQObject oOut = MQ_CreateObject();
  char objname[151];
  doc->GetUnusedObjectName(objname, 150, "draw");
  oOut->SetName(objname);
  
  MQObject mqoCacheRoot = _FindMQObjectByName(doc, "MQDrawPolygonTexCache");
  */
  _MyObject _oOut;
  MyObject oOut = &_oOut;
  
  CRaster2Vector conv(opt);
  
    
  int numV = oSrc->GetVertexCount();
  int numF = oSrc->GetFaceCount();
  for(int fi=0;fi<numF;fi++)
  {
    int numFV = oSrc->GetFacePointCount(fi);
    if(numFV!=3)continue;
    
    CacheTexInfo *pTexInfo = texManager.GetTexture(oSrc, fi);
    if(pTexInfo==NULL)continue;
    //MyObject mqoTex = pTexInfo->o;
    
    std::vector<MyPoint> ptsTri(numFV);
    std::vector<MyCoordinate> coordTri(numFV);
    GetPointAndCoord(oSrc, fi, numFV, vidxTri, ptsTri, coordTri);
    
    conv.OutputToMetaseqFast_LineTri(oOut, *pTexInfo, ptsTri, coordTri, opt.zscalemode, opt.zscale);
    
    conv.reset();
  }
  
  /*
  if(bOptimize)
  {
    MyOutputDebugStringA("OptimizeVertex\n");
    oOut->OptimizeVertex(0.0f, NULL);
  }
  MyOutputDebugStringA("AddObject\n");
  doc->AddObject(oOut);
  */
  
  conv.reset();
  
  oOut->write(fpOut);
  fclose(fpOut);
  
  //const char* source_window = "Source";
  //cv::namedWindow( source_window, cv::WINDOW_NORMAL );
  //imshow( source_window, dstEdge );
  //cv::waitKey(0);
  MyOutputDebugStringA("End\n");
  
  return true;
}


bool MakeTexCache(const char *texFullPath, const char *outPath, DrawPolygonOptions &opt)
{
  cv::Mat mat = cv::imread(texFullPath);
  if(mat.empty())return false;
  
  
  //cv::Mat &mat = t.matRaster;
  CRaster2Vector conv(opt);
  if(conv.Raster2Vector(mat)==false)return false;
  
  /*
  MQObject o = MQ_CreateObject();
  o->SetName(cachename);
  */
  
  double wf = mat.cols, hf = mat.rows;
  
  _MyObject _out;
  MyObject out = &_out;
  
  cv::Mat uvMat(2, 3, CV_64F, cv::Scalar(0));
  uvMat.at<double>(0,0) = 1.0/wf;
  uvMat.at<double>(1,1) = 1.0/hf;
  transform3 invMat =  flipX()*translate(Vector3(0.0, -hf, 0.0));
  if(opt.bFacegen)conv.OutputToMetaseq_LineTri(out, uvMat, opt.bThresholdMennuki, &invMat, true/*bInvertFace*/, opt.matid);
  else        conv.OutputToMetaseq_LineOnly(out);
  
  
  FILE *fp = fopen(outPath, "wb");
  if(fp==NULL)return false;
  
  out->write(fp);
  
  fclose(fp);
  
  conv.reset();
  return true;
}

#include <boost/program_options.hpp>
#include <iostream>

int main(int argc, char** argv)
{
  namespace po = boost::program_options;
  
  DrawPolygonOptions opt;
  /*
  int convmode;
  int zscalemode;
  double zscale;
  int edgeproc;
  int vectorconv;
  double threshold;
  int np;
  bool facegen;
  bool mennuki;
  double thresholdMennuki;
  bool opt;
  */
  po::options_description desc("options:");
  desc.add_options()
    ("help", "help")
    ("mode,m", po::value<int>(&opt.convmode)->default_value(1), "変換モード:\n\t0: コピペ画像 (Windowsのみ)\n\t1: 面テクスチャ/高速\n\t2: 面テクスチャ/低速\n\t3: 画像入力")
    ("in,i", po::value< std::string >(), "input file")
    ("texCache,t", po::value< std::string >(), "texCache input file")
    ("out,o", po::value< std::string >(), "output file")
    ("zscalemode,a", po::value<int>(&opt.zscalemode)->default_value(1), "立体テクスチャのZ伸縮:\n\t0: 伸縮しない\n\t1: 自動\n\t2: 固定値")
    ("zscale,z", po::value<double>(&opt.zscale)->default_value(1.0), "立体テクスチャのZ伸縮率")
    ("edgeproc,e", po::value<int>(&opt.edgeproc)->default_value(0), "ライン抽出:\n\t0: Canny\n\t1: Laplacian\n\t2: Sobel\n\t3: 無し（中央線）")
    ("vectorize,v", po::value<int>(&opt.vectorconv)->default_value(0), "ベクトル化:\n\t0: モノクロ値を重みとして使用\n\t1: 2値化（重み無し）")
    ("threshold,b", po::value<double>(&opt.threshold)->default_value(0.5), "2値化の判定値")
    ("np,n", po::value<int>(&opt.np)->default_value(100), "出力頂点数")
    ("facegen,g", po::bool_switch(&opt.bFacegen), "面を生成")
    ("mennuki,h", po::bool_switch(&opt.bThresholdMennuki), "面抜きする")
    ("tm,c", po::value<double>(&opt.thresholdMennuki)->default_value(0.5), "面抜き判定値")
    ("opt,p", po::bool_switch(&opt.opt), "近接する頂点を接合")
    ("matid,x", po::value<int>(&opt.matid)->default_value(-1), "マテリアルのインデックス番号")
  ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }
  
  if (!vm.count("out"))
  {
    std::cout << "出力ファイルが指定されていません\n";
    return -1;
  }
  
  /*
  if (vm.count("in"))
  {
    std::cout << "入力ファイル " 
      << vm["in"].as<std::string>() << ".\n";
  } else {
    std::cout << "入力ファイルが指定されていません\n";
    return 1;
  }
  */
  /*
  if (vm.count("zscalemode"))
  {
    opt.zscalemode = vm["in"].as<std::string>();
  }*/
  
  
  
  
  bool bRet = false;
  switch(opt.convmode)
  {
  case 0:
  default:
#if defined _WIN32 || defined __CYGWIN__
    bRet = ConvertFromClipboard(opt, vm["out"].as<std::string>().c_str());
#endif
    break;
  case 1:
    if (vm.count("in") && vm.count("texCache")/* && vm.count("out")*/)
    {
      MQTexManager texManager;
      bRet = texManager.LoadDPMV(vm["texCache"].as<std::string>().c_str());
      if(!bRet)break;
      _MyObject oSrc;
      bRet = oSrc.read(vm["in"].as<std::string>().c_str());
      if(!bRet)break;
      bRet = ConvertFromTextureFast(opt, &oSrc, texManager, vm["out"].as<std::string>().c_str());
    }
    break;
  case 2:
    if (vm.count("in") && vm.count("texCache")/* && vm.count("out")*/)
    {
      MQTexManager texManager;
      bRet = texManager.LoadDPMR(vm["texCache"].as<std::string>().c_str());
      if(!bRet)break;
      _MyObject oSrc;
      bRet = oSrc.read(vm["in"].as<std::string>().c_str());
      if(!bRet)break;
      bRet = ConvertFromTextureSlow(opt, &oSrc, texManager, vm["out"].as<std::string>().c_str());
    }
    break;
  case 3:
    if (vm.count("in")/* && vm.count("out")*/)
    {
      bRet = MakeTexCache(vm["in"].as<std::string>().c_str(), vm["out"].as<std::string>().c_str(), opt);
    }
    break;
  }
  
  return bRet ? 0 : -1;
}
