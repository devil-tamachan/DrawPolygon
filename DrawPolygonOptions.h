
#ifndef _TAMADRAWPOLYOPT_
#define _TAMADRAWPOLYOPT_



class DrawPolygonOptions
{
public:
  DrawPolygonOptions()
  {
    convmode = 0;
    zscalemode = 1;
    zscale = 1.0;
    edgeproc = 0;
    vectorconv = 0;
    threshold = 0.5;
    np = 100;
    bFacegen = true;
    bThresholdMennuki = true;
    thresholdMennuki = 0.5;
    opt = true;
    matid = -1;
  }
  ~DrawPolygonOptions()
  {
  }

  int convmode;
  int zscalemode;
  double zscale;
  int edgeproc;
  int vectorconv;
  double threshold;
  int np;
  bool bFacegen;
  bool bThresholdMennuki;
  double thresholdMennuki;
  bool opt;
  int matid;
};


#endif //_TAMADRAWPOLYOPT_
