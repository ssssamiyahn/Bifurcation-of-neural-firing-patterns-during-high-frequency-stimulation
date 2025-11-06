NEURON {
	SUFFIX calvrec
  RANGE rx_rec, er
	RANGE x, y, z
	: GLOBAL is
	POINTER im
}

PARAMETER {
	: default transfer resistance between stim electrodes and axon
	: rx_stim = 0 (megohm) : mV/nA
	rx_rec = 1 (megohm) : mV/nA
	: rx_back = 0 (megohm) : mV/nA
	x = 0 (1) : spatial coords
	y = 0 (1)
	z = 0 (1)
}

ASSIGNED {
	v (millivolts)
	: is (milliamp)
	ex (millivolts)
	im (milliamp/cm2)
	er (microvolts)
	area (micron2)
}

INITIAL {
  : ex = is*rx_stim*(1e3)
  : ex = is*rx_stim*(1e3)
  er = (10)*rx_rec*im*area
: this demonstrates that area is known
: UNITSOFF
: printf("area = %f\n", area)
: printf("er = %f\n", er)
: printf("rx_rec = %f\n", rx_rec)
: printf("im = %f\n", im)
: printf("area = %f\n", area)
: UNITSON
}

: Use BREAKPOINT for NEURON 5.4 and earlier
: BREAKPOINT {
:	SOLVE f METHOD cvode_t
: }
:
: PROCEDURE f() {
:	: 1 mA * 1 megohm is 1000 volts
:	: but ex is in mV
:	ex = is*rx*(1e6)
:	er = (10)*rx*im*area
: }

: With NEURON 5.5 and later, abandon the BREAKPOINT block and PROCEDURE f(),
: and instead use BEFORE BREAKPOINT and AFTER SOLVE

: BEFORE BREAKPOINT { : before each cy' = f(y,t) setup
  :ex = is*rx_stim*(1e3)
:   ex = is*rx_stim*(1e3)
: }
AFTER SOLVE { : after each solution step
  er = (10)*rx_rec*im*area
 }

