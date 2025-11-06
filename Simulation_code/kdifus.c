/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 static void _difusfunc(ldifusfunc2_t, NrnThread*);
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__kdifus
#define _nrn_initial _nrn_initial__kdifus
#define nrn_cur _nrn_cur__kdifus
#define _nrn_current _nrn_current__kdifus
#define nrn_jacob _nrn_jacob__kdifus
#define nrn_state _nrn_state__kdifus
#define _net_receive _net_receive__kdifus 
#define conc conc__kdifus 
#define f_rates f_rates__kdifus 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define k2buf _p[0]
#define k2buf_columnindex 0
#define TotalBuffer _p[1]
#define TotalBuffer_columnindex 1
#define width _p[2]
#define width_columnindex 2
#define r _p[3]
#define r_columnindex 3
#define kbath _p[4]
#define kbath_columnindex 4
#define Dk _p[5]
#define Dk_columnindex 5
#define sp _p[6]
#define sp_columnindex 6
#define rseg _p[7]
#define rseg_columnindex 7
#define KBuffer _p[8]
#define KBuffer_columnindex 8
#define Buffer _p[9]
#define Buffer_columnindex 9
#define ik _p[10]
#define ik_columnindex 10
#define Bufferspace _p[11]
#define Bufferspace_columnindex 11
#define k1buf _p[12]
#define k1buf_columnindex 12
#define kd _p[13]
#define kd_columnindex 13
#define B0 _p[14]
#define B0_columnindex 14
#define ko _p[15]
#define ko_columnindex 15
#define Dko _p[16]
#define Dko_columnindex 16
#define DKBuffer _p[17]
#define DKBuffer_columnindex 17
#define DBuffer _p[18]
#define DBuffer_columnindex 18
#define v _p[19]
#define v_columnindex 19
#define _g _p[20]
#define _g_columnindex 20
#define _ion_ik	*_ppvar[0]._pval
#define _ion_ko	*_ppvar[1]._pval
#define _style_k	*((int*)_ppvar[2]._pvoid)
#define _ion_dikdv	*_ppvar[3]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_f_rates(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_kdifus", _hoc_setdata,
 "f_rates_kdifus", _hoc_f_rates,
 0, 0
};
 /* declare global and static user variables */
 static int _thread1data_inuse = 0;
static double _thread1data[2];
#define _gth 2
#define crossSectionalArea_kdifus _thread1data[0]
#define crossSectionalArea _thread[_gth]._pval[0]
#define extracellularVolumePerLength_kdifus _thread1data[1]
#define extracellularVolumePerLength _thread[_gth]._pval[1]
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "extracellularVolumePerLength_kdifus", "um2",
 "crossSectionalArea_kdifus", "um2",
 "TotalBuffer_kdifus", "mM",
 "width_kdifus", "um",
 "Dk_kdifus", "um2/ms",
 "sp_kdifus", "um",
 "rseg_kdifus", "um",
 "KBuffer_kdifus", "mM",
 "Buffer_kdifus", "mM",
 0,0
};
 static double Buffer0 = 0;
 static double KBuffer0 = 0;
 static double delta_t = 0.01;
 static double ko0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "extracellularVolumePerLength_kdifus", &extracellularVolumePerLength_kdifus,
 "crossSectionalArea_kdifus", &crossSectionalArea_kdifus,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(NrnThread*, _Memb_list*, int);
static void nrn_state(NrnThread*, _Memb_list*, int);
 static void nrn_cur(NrnThread*, _Memb_list*, int);
static void  nrn_jacob(NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(NrnThread*, _Memb_list*, int);
static void _ode_matsol(NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[4]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"kdifus",
 "k2buf_kdifus",
 "TotalBuffer_kdifus",
 "width_kdifus",
 "r_kdifus",
 "kbath_kdifus",
 "Dk_kdifus",
 "sp_kdifus",
 "rseg_kdifus",
 0,
 0,
 "KBuffer_kdifus",
 "Buffer_kdifus",
 0,
 0};
 static Symbol* _k_sym;
 static int _type_ik;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 21, _prop);
 	/*initialize range parameters*/
 	k2buf = 0.0008;
 	TotalBuffer = 500;
 	width = 1;
 	r = -1.15;
 	kbath = 15;
 	Dk = 1.85;
 	sp = 0.01;
 	rseg = 1.7;
 	_prop->param = _p;
 	_prop->param_size = 21;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 5, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_k_sym);
  _type_ik = prop_ion->_type;
 nrn_check_conc_write(_prop, prop_ion, 0);
 nrn_promote(prop_ion, 3, 0);
 	_ppvar[0]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[1]._pval = &prop_ion->param[2]; /* ko */
 	_ppvar[2]._pvoid = (void*)(&(prop_ion->dparam[0]._i)); /* iontype for k */
 	_ppvar[3]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _thread_mem_init(Datum*);
 static void _thread_cleanup(Datum*);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _kdifus_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("k", -10000.);
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 4);
  _extcall_thread = (Datum*)ecalloc(3, sizeof(Datum));
  _thread_mem_init(_extcall_thread);
  _thread1data_inuse = 0;
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 1, _thread_mem_init);
     _nrn_thread_reg(_mechtype, 0, _thread_cleanup);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 21, 5);
  hoc_register_dparam_semantics(_mechtype, 0, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "#k_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "cvodeieq");
 	nrn_writes_conc(_mechtype, 0);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_ldifus1(_difusfunc);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 kdifus kdifus.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 
#define FARADAY _nrnunit_FARADAY[_nrnunit_use_legacy_]
static double _nrnunit_FARADAY[2] = {0x1.78e555060882cp+16, 96485.3}; /* 96485.3321233100141 */
 
#define PI _nrnunit_PI[_nrnunit_use_legacy_]
static double _nrnunit_PI[2] = {0x1.921fb54442d18p+1, 3.14159}; /* 3.14159265358979312 */
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int f_rates(_threadargsproto_);
 extern double *_nrn_thread_getelm(SparseObj*, int, int);
 
#define _MATELM1(_row,_col) *(_nrn_thread_getelm(_so, _row + 1, _col + 1))
 
#define _RHS1(_arg) _rhs[_arg+1]
  
#define _linmat1  0
 static int _spth1 = 1;
 static int _cvspth1 = 0;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[3], _dlist1[3]; static double *_temp1;
 static int conc();
 
static int conc (void* _so, double* _rhs, double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt)
 {int _reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<3;_i++){
  	_RHS1(_i) = -_dt1*(_p[_slist1[_i]] - _p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
}  
_RHS1(2) *= ( extracellularVolumePerLength) ;
_MATELM1(2, 2) *= ( extracellularVolumePerLength);  }
 /* COMPARTMENT extracellularVolumePerLength {
     ko }
   */
 /* LONGITUDINAL_DIFFUSION Dk * crossSectionalArea {
     ko }
   */
 /* ~ ko < < ( ik * 2.0 * PI * rseg * ( 1e4 ) / ( FARADAY ) )*/
 f_flux = b_flux = 0.;
 _RHS1( 2) += (b_flux =   ( ik * 2.0 * PI * rseg * ( 1e4 ) / ( FARADAY ) ) );
 /*FLUX*/
  f_rates ( _threadargs_ ) ;
   /* ~ ko + Buffer <-> KBuffer ( k1buf , k2buf )*/
 f_flux =  k1buf * Buffer * ko ;
 b_flux =  k2buf * KBuffer ;
 _RHS1( 0) -= (f_flux - b_flux);
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  k1buf * ko ;
 _MATELM1( 0 ,0)  += _term;
 _MATELM1( 2 ,0)  += _term;
 _MATELM1( 1 ,0)  -= _term;
 _term =  k1buf * Buffer ;
 _MATELM1( 0 ,2)  += _term;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  k2buf ;
 _MATELM1( 0 ,1)  -= _term;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
    } return _reset;
 }
 
static int  f_rates ( _threadargsproto_ ) {
   k1buf = k2buf / ( 1.0 + exp ( ( ko - kbath ) / ( r ) ) ) ;
    return 0; }
 
static void _hoc_f_rates(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 f_rates ( _p, _ppvar, _thread, _nt );
 hoc_retpushx(_r);
}
 
/*CVODE ode begin*/
 static int _ode_spec1(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<3;_i++) _p[_dlist1[_i]] = 0.0;}
 /* COMPARTMENT extracellularVolumePerLength {
   ko }
 */
 /* LONGITUDINAL_DIFFUSION Dk * crossSectionalArea {
   ko }
 */
 /* ~ ko < < ( ik * 2.0 * PI * rseg * ( 1e4 ) / ( FARADAY ) )*/
 f_flux = b_flux = 0.;
 Dko += (b_flux =   ( ik * 2.0 * PI * rseg * ( 1e4 ) / ( FARADAY ) ) );
 /*FLUX*/
  f_rates ( _threadargs_ ) ;
 /* ~ ko + Buffer <-> KBuffer ( k1buf , k2buf )*/
 f_flux =  k1buf * Buffer * ko ;
 b_flux =  k2buf * KBuffer ;
 DBuffer -= (f_flux - b_flux);
 Dko -= (f_flux - b_flux);
 DKBuffer += (f_flux - b_flux);
 
 /*REACTION*/
  _p[_dlist1[2]] /= ( extracellularVolumePerLength);
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1(void* _so, double* _rhs, double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {int _reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<3;_i++){
  	_RHS1(_i) = _dt1*(_p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
}  
_RHS1(2) *= ( extracellularVolumePerLength) ;
_MATELM1(2, 2) *= ( extracellularVolumePerLength);  }
 /* COMPARTMENT extracellularVolumePerLength {
 ko }
 */
 /* LONGITUDINAL_DIFFUSION Dk * crossSectionalArea {
 ko }
 */
 /* ~ ko < < ( ik * 2.0 * PI * rseg * ( 1e4 ) / ( FARADAY ) )*/
 /*FLUX*/
  f_rates ( _threadargs_ ) ;
 /* ~ ko + Buffer <-> KBuffer ( k1buf , k2buf )*/
 _term =  k1buf * ko ;
 _MATELM1( 0 ,0)  += _term;
 _MATELM1( 2 ,0)  += _term;
 _MATELM1( 1 ,0)  -= _term;
 _term =  k1buf * Buffer ;
 _MATELM1( 0 ,2)  += _term;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 1 ,2)  -= _term;
 _term =  k2buf ;
 _MATELM1( 0 ,1)  -= _term;
 _MATELM1( 2 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
    } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 3;}
 
static void _ode_spec(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ik = _ion_ik;
  ko = _ion_ko;
  ko = _ion_ko;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
  _ion_ko = ko;
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 3; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 	_pv[2] = &(_ion_ko);
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _cvode_sparse_thread(&_thread[_cvspth1]._pvoid, 3, _dlist1, _p, _ode_matsol1, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ik = _ion_ik;
  ko = _ion_ko;
  ko = _ion_ko;
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _thread_mem_init(Datum* _thread) {
  if (_thread1data_inuse) {_thread[_gth]._pval = (double*)ecalloc(2, sizeof(double));
 }else{
 _thread[_gth]._pval = _thread1data; _thread1data_inuse = 1;
 }
 }
 
static void _thread_cleanup(Datum* _thread) {
   _nrn_destroy_sparseobj_thread(_thread[_cvspth1]._pvoid);
   _nrn_destroy_sparseobj_thread(_thread[_spth1]._pvoid);
  if (_thread[_gth]._pval == _thread1data) {
   _thread1data_inuse = 0;
  }else{
   free((void*)_thread[_gth]._pval);
  }
 }
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_k_sym, _ppvar, 0, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 1, 2);
   nrn_update_ion_pointer(_k_sym, _ppvar, 3, 4);
 }
 static void* _difspace1;
extern double nrn_nernst_coef();
static double _difcoef1(int _i, double* _p, Datum* _ppvar, double* _pdvol, double* _pdfcdc, Datum* _thread, NrnThread* _nt) {
   *_pdvol =  extracellularVolumePerLength ;
 if (_i == 0) {
  *_pdfcdc = nrn_nernst_coef(_type_ik)*( ( _ion_dikdv  * 2.0 * PI * rseg * ( 1e4 ) / ( FARADAY ) ));
 }else{ *_pdfcdc=0.;}
   return Dk * crossSectionalArea ;
}
 static void _difusfunc(ldifusfunc2_t _f, NrnThread* _nt) {int _i;
  (*_f)(_mechtype, _difcoef1, &_difspace1, 0,  -2, 16 , _nt);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
  int _i; double _save;{
  Buffer = Buffer0;
  KBuffer = KBuffer0;
 {
   extracellularVolumePerLength = PI * ( pow( ( rseg + sp ) , 2.0 ) - pow( rseg , 2.0 ) ) ;
   crossSectionalArea = extracellularVolumePerLength ;
   kd = 1.0 / ( 1.0 + exp ( ( ko - kbath ) / ( r ) ) ) ;
   B0 = TotalBuffer / ( 1.0 + kd * ko ) ;
   Buffer = B0 ;
   KBuffer = TotalBuffer - B0 ;
   }
 
}
}

static void nrn_init(NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  ik = _ion_ik;
  ko = _ion_ko;
  ko = _ion_ko;
 initmodel(_p, _ppvar, _thread, _nt);
  _ion_ko = ko;
  nrn_wrote_conc(_k_sym, (&(_ion_ko)) - 2, _style_k);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt, double _v){double _current=0.;v=_v;{
} return _current;
}

static void nrn_cur(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 
}
 
}

static void nrn_jacob(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
double _dtsav = dt;
if (secondorder) { dt *= 0.5; }
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
  ik = _ion_ik;
  ko = _ion_ko;
  ko = _ion_ko;
 {  sparse_thread(&_thread[_spth1]._pvoid, 3, _slist1, _dlist1, _p, &t, dt, conc, _linmat1, _ppvar, _thread, _nt);
     if (secondorder) {
    int _i;
    for (_i = 0; _i < 3; ++_i) {
      _p[_slist1[_i]] += dt*_p[_dlist1[_i]];
    }}
 } {
   }
  _ion_ko = ko;
}}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = Buffer_columnindex;  _dlist1[0] = DBuffer_columnindex;
 _slist1[1] = KBuffer_columnindex;  _dlist1[1] = DKBuffer_columnindex;
 _slist1[2] = ko_columnindex;  _dlist1[2] = Dko_columnindex;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "kdifus.mod";
static const char* nmodl_file_text = 
  "COMMENT\n"
  "Longitudinal diffusion of potassium from bellinger et al. 2008\n"
  "add glial cells uptake potassium\n"
  "modeled as potassium buffering\n"
  "refer to articles of H. KAGER et al. 2000, M. Bazhenov et al. 2004, Flavio Fro\n"
  "\n"
  "hlich et al. 2006, Naomi Lewin et al. 2012\n"
  "Zheng Lyup, 2018\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX kdifus\n"
  "	USEION k READ ik, ko WRITE ko\n"
  "	:USEION k READ ik WRITE ko\n"
  "	RANGE TotalBuffer, k2buf, r, width, kbath\n"
  "	RANGE Dk, sp, rseg, length\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "  (molar) = (1/liter)\n"
  "  (mM) = (millimolar)\n"
  "	(um) = (micron)\n"
  "	(mA) = (milliamp)\n"
  "	FARADAY = (faraday) (coulomb)\n"
  "	PI = (pi) (1)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	k2buf=0.0008\n"
  "	TotalBuffer=500(mM)\n"
  "	width=1(um)\n"
  "	r=-1.15\n"
  "	kbath=15\n"
  "                	\n"
  "                Dk = 1.85 (um2/ms)\n"
  "	sp = 0.01 (um)\n"
  "	rseg = 1.7 (um)\n"
  "	extracellularVolumePerLength (um2)\n"
  "	crossSectionalArea (um2)\n"
  "}\n"
  "\n"
  "ASSIGNED { \n"
  "   ik (mA/cm2)\n"
  "   :ko(mM)\n"
  "   Bufferspace(1)\n"
  "   \n"
  "   k1buf\n"
  "   \n"
  "   kd (/mM)  :dissociation constant for the buffer\n"
  "   B0 (mM)   :the initial value of the free buffer\n"
  "    }\n"
  "\n"
  "STATE {  \n"
  "      ko (mM)\n"
  "      \n"
  "      KBuffer(mM)\n"
  "      Buffer(mM)\n"
  "       \n"
  "  }\n"
  "\n"
  "INITIAL {\n"
  "	extracellularVolumePerLength = PI * ( (rseg+sp)^2 - rseg^2 ) \n"
  "	crossSectionalArea = extracellularVolumePerLength\n"
  "	\n"
  "	\n"
  "	kd=1/(1+exp((ko-kbath)/(r)))\n"
  "	B0=TotalBuffer/(1+kd*ko)\n"
  "	\n"
  "	Buffer = B0\n"
  "	KBuffer=TotalBuffer-B0\n"
  "		\n"
  "}\n"
  "\n"
  "\n"
  "BREAKPOINT {\n"
  "  SOLVE conc METHOD sparse\n"
  "	\n"
  "}\n"
  "\n"
  "\n"
  "KINETIC conc {\n"
  "	COMPARTMENT extracellularVolumePerLength {ko}\n"
  "	LONGITUDINAL_DIFFUSION Dk*crossSectionalArea {ko}\n"
  "\n"
  "  ~ ko << (ik*2*PI*rseg*(1e4)/(FARADAY))  : with or without K+ accumulation\n"
  "  \n"
  "  \n"
  "  f_rates()\n"
  "  ~ ko + Buffer <-> KBuffer (k1buf, k2buf)\n"
  "}\n"
  "\n"
  "\n"
  " PROCEDURE f_rates(){\n"
  " k1buf= k2buf / (1+exp((ko-kbath)/(r)))\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  ;
#endif
