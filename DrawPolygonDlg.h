
#ifndef _TAMADRAWPOLYDLG_
#define _TAMADRAWPOLYDLG_



class DrawPolygonDialog : public MQDialog
{
public:
  DrawPolygonDialog(MQWindowBase& parent);
  ~DrawPolygonDialog();


  BOOL uiChanged(MQWidgetBase *sender, MQDocument doc)
  {
    UpdateEnable(doc);
    return FALSE;
  }
  
  void UpdateEnable(MQDocument doc)
  {
    MakeObjList(doc);
    bool bCenterLine = combo_edgeproc->GetCurrentIndex()==3 ? false:true;
    bool bFaceGen = check_facegen->GetChecked();
    bool bThresholdMennuki = check_thresholdMennuki->GetChecked();
    slider_thresholdMennuki->SetEnabled(bCenterLine && bFaceGen && bThresholdMennuki);
    lbl_slider_thresholdMennuki->SetEnabled(slider_thresholdMennuki->GetEnabled());
    
    bool bConvModeFast = combo_src->GetCurrentIndex()==1;
    combo_zscale->SetEnabled(bConvModeFast);
    lbl_combo_zscale->SetEnabled(combo_zscale->GetEnabled());
    
    bool bZScaleModeInputValue = combo_zscale->GetCurrentIndex()==2;
    dblspin_zscale->SetEnabled(bZScaleModeInputValue);
    lbl_dblspin_zscale->SetEnabled(dblspin_zscale->GetEnabled());
    
    check_thresholdMennuki->SetEnabled(bCenterLine && bFaceGen);
    lbl_check_thresholdMennuki->SetEnabled(check_thresholdMennuki->GetEnabled());
    
    slider_threshold->SetEnabled( combo_vectorconv->GetCurrentIndex()==0 ? false:true);
    lbl_slider_threshold->SetEnabled(slider_threshold->GetEnabled());
    
    bool bClipboard = combo_src->GetCurrentIndex()==0 ? true : false;
    combo_filterobj->SetEnabled(!bClipboard);
    lbl_combo_filterobj->SetEnabled(combo_filterobj->GetEnabled());
    
    check_visibleObjOnly->SetEnabled(combo_filterobj->GetEnabled());
    lbl_check_visibleObjOnly->SetEnabled(check_visibleObjOnly->GetEnabled());
  }

  void MakeObjList(MQDocument doc)
  {
    bool bVisibleObjOnly = check_visibleObjOnly->GetChecked();
    
    combo_filterobj->ClearItems();
    combo_filterobj->AddItem(L"���ׂẴI�u�W�F�N�g");
    m_objIdx.clear();
    int numObj = doc->GetObjectCount();
    for(int i=0;i<numObj;i++)
    {
      MQObject o = doc->GetObject(i);
      if(o==NULL)continue;
      if(bVisibleObjOnly && o->GetVisible()==0)continue;
      m_objIdx.push_back(i);
      combo_filterobj->AddItem(o->GetNameW());
    }
  }

  int GetSelectObjIdx()
  {
    int i = combo_filterobj->GetCurrentIndex() - 1;
    if(i>=0 && i<m_objIdx.size())return m_objIdx[i];
    return -1;
  }

  MQComboBox *combo_src;
  MQComboBox *combo_filterobj;
  MQLabel *lbl_combo_filterobj;
  MQCheckBox *check_visibleObjOnly;
  MQLabel *lbl_check_visibleObjOnly;
  MQComboBox *combo_zscale;
  MQLabel *lbl_combo_zscale;
  MQDoubleSpinBox *dblspin_zscale;
  MQLabel *lbl_dblspin_zscale;
  MQComboBox *combo_edgeproc;
  MQComboBox *combo_vectorconv;
  MQSlider *slider_threshold;
  MQLabel *lbl_slider_threshold;
  MQSpinBox *spin_np;
  MQCheckBox *check_facegen;
  MQSlider *slider_thresholdMennuki;
  MQLabel *lbl_slider_thresholdMennuki;
  MQCheckBox *check_thresholdMennuki;
  MQLabel *lbl_check_thresholdMennuki;
  MQCheckBox *check_optimize;
  std::vector<int> m_objIdx;
};

DrawPolygonDialog::DrawPolygonDialog(MQWindowBase& parent) : MQDialog(parent)
{
  SetTitle(L"DrawPolygon");

  MQFrame *mainFrame = CreateHorizontalFrame(this);

  MQFrame *paramFrame = CreateHorizontalFrame(mainFrame);
  paramFrame->SetMatrixColumn(2);
  
  CreateLabel(paramFrame, L"���͉摜");
  combo_src = CreateComboBox(paramFrame);
  combo_src->AddItem(L"�R�s�y�摜");
  combo_src->AddItem(L"�ʃe�N�X�`��/����");
  combo_src->AddItem(L"�ʃe�N�X�`��/�ᑬ");
  combo_src->SetCurrentIndex(0);
  combo_src->AddChangedEvent(this, &DrawPolygonDialog::uiChanged);
  
  lbl_combo_filterobj = CreateLabel(paramFrame, L"���̓I�u�W�F�N�g");
  combo_filterobj = CreateComboBox(paramFrame);
  combo_filterobj->SetCurrentIndex(0);

  lbl_check_visibleObjOnly = CreateLabel(paramFrame, L"\"���̓I�u�W�F�N�g\"�����\���I�u�W�F������");
  check_visibleObjOnly = CreateCheckBox(paramFrame);
  check_visibleObjOnly->SetChecked(true);
  check_visibleObjOnly->AddChangedEvent(this, &DrawPolygonDialog::uiChanged);
  
  lbl_combo_zscale = CreateLabel(paramFrame, L"���̃e�N�X�`����Z�L�k");
  combo_zscale = CreateComboBox(paramFrame);
  combo_zscale->AddItem(L"�L�k���Ȃ�");
  combo_zscale->AddItem(L"����");
  combo_zscale->AddItem(L"�Œ�l");
  combo_zscale->SetCurrentIndex(1);
  combo_zscale->AddChangedEvent(this, &DrawPolygonDialog::uiChanged);
  
  lbl_dblspin_zscale = CreateLabel(paramFrame, L"���̃e�N�X�`����Z�L�k��");
  dblspin_zscale = CreateDoubleSpinBox(paramFrame);
  dblspin_zscale->SetMin(0.0000001);
  dblspin_zscale->SetPosition(1.0);
  
  CreateLabel(paramFrame, L"���C�����o");
  combo_edgeproc = CreateComboBox(paramFrame);
  combo_edgeproc->AddItem(L"Canny (��������)");
  combo_edgeproc->AddItem(L"Laplacian");
  combo_edgeproc->AddItem(L"Sobel");
  combo_edgeproc->AddItem(L"�����i�������j");
  combo_edgeproc->SetCurrentIndex(0);
  combo_edgeproc->AddChangedEvent(this, &DrawPolygonDialog::uiChanged);
  
  CreateLabel(paramFrame, L"�x�N�g����");
  combo_vectorconv = CreateComboBox(paramFrame);
  combo_vectorconv->AddItem(L"���m�N���l���d�݂Ƃ��Ďg�p");
  combo_vectorconv->AddItem(L"2�l���i�d�ݖ����j");
  combo_vectorconv->SetCurrentIndex(0);
  combo_vectorconv->AddChangedEvent(this, &DrawPolygonDialog::uiChanged);
  
  lbl_slider_threshold = CreateLabel(paramFrame, L"2�l���̔���l");
  slider_threshold = CreateSlider(paramFrame);
  slider_threshold->SetMin(0.01);
  slider_threshold->SetMax(0.99);
  slider_threshold->SetPosition(0.5);
  slider_threshold->SetEnabled(false);
  
  CreateLabel(paramFrame, L"�o�͒��_��");
  spin_np = CreateSpinBox(paramFrame);
  spin_np->SetMin(2);
  spin_np->SetMax(1000);
  spin_np->SetPosition(100);

  CreateLabel(paramFrame, L"�ʂ𐶐�");
  check_facegen = CreateCheckBox(paramFrame);
  check_facegen->SetChecked(true);
  check_facegen->AddChangedEvent(this, &DrawPolygonDialog::uiChanged);

  lbl_check_thresholdMennuki = CreateLabel(paramFrame, L"�ʔ�������");
  check_thresholdMennuki = CreateCheckBox(paramFrame);
  check_thresholdMennuki->SetChecked(true);
  check_thresholdMennuki->AddChangedEvent(this, &DrawPolygonDialog::uiChanged);
  
  
  lbl_slider_thresholdMennuki = CreateLabel(paramFrame, L"�ʔ�������l (�����c�遨)");
  slider_thresholdMennuki = CreateSlider(paramFrame);
  slider_thresholdMennuki->SetMin(0.01);
  slider_thresholdMennuki->SetMax(0.99);
  slider_thresholdMennuki->SetPosition(0.5);
  slider_thresholdMennuki->SetEnabled(true);

  CreateLabel(paramFrame, L"�ߐڂ��钸�_��ڍ�");
  check_optimize = CreateCheckBox(paramFrame);
  check_optimize->SetChecked(true);

  MQFrame *sideFrame = CreateVerticalFrame(mainFrame);

  MQButton *okbtn = CreateButton(sideFrame, L"OK");
  okbtn->SetDefault(true);
  okbtn->SetModalResult(MQDialog::DIALOG_OK);

  MQButton *cancelbtn = CreateButton(sideFrame, L"Cancel");
  cancelbtn->SetCancel(true);
  cancelbtn->SetModalResult(MQDialog::DIALOG_CANCEL);
}

DrawPolygonDialog::~DrawPolygonDialog()
{
}

#endif //_TAMADRAWPOLYDLG_
