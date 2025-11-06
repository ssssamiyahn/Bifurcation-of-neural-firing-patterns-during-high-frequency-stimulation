#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _Nakpump_reg();
extern void _ca_reg();
extern void _calvrec_reg();
extern void _fastK_reg();
extern void _kca_reg();
extern void _kdifrl_reg();
extern void _kdifus_reg();
extern void _kma_reg();
extern void _kms_reg();
extern void _ksteady_reg();
extern void _kv_reg();
extern void _na12_reg();
extern void _na16_reg();
extern void _nax_reg();
extern void _xtra_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," Nakpump.mod");
fprintf(stderr," ca.mod");
fprintf(stderr," calvrec.mod");
fprintf(stderr," fastK.mod");
fprintf(stderr," kca.mod");
fprintf(stderr," kdifrl.mod");
fprintf(stderr," kdifus.mod");
fprintf(stderr," kma.mod");
fprintf(stderr," kms.mod");
fprintf(stderr," ksteady.mod");
fprintf(stderr," kv.mod");
fprintf(stderr," na12.mod");
fprintf(stderr," na16.mod");
fprintf(stderr," nax.mod");
fprintf(stderr," xtra.mod");
fprintf(stderr, "\n");
    }
_Nakpump_reg();
_ca_reg();
_calvrec_reg();
_fastK_reg();
_kca_reg();
_kdifrl_reg();
_kdifus_reg();
_kma_reg();
_kms_reg();
_ksteady_reg();
_kv_reg();
_na12_reg();
_na16_reg();
_nax_reg();
_xtra_reg();
}
