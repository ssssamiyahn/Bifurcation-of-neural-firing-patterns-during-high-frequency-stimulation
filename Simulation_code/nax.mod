TITLE na3
: Na current for axon. No slow inact.
: M.Migliore Jul. 1997

NEURON {
	SUFFIX nax
	USEION na READ ena WRITE ina
	RANGE  gbar, minf, hinf,mtau, htau,thinf, qinf,thegna,gl,el,h0
	
}

PARAMETER {
	gbar 	(mho/cm2)	
	gl	(mho/cm2)
	el	(mV)   							
	:tha  =  -30	(mV)		: v 1/2 for act	
       tha  =  -30	(mV)		: v 1/2 for act	
	qa   = 7.5	(mV)		: act slope (4.5)		
	Ra  	(/ms)		: open (v)		
	Rb   (/ms)		: close (v)		

	:thi1  = -45	(mV)		: v 1/2 for inact 	
	:thi2  = -42 	(mV)		: v 1/2 for inact 
      thi1  = -45(mV)		: v 1/2 for inact 	
	thi2  = -45 	(mV)		: v 1/2 for inact	
	qd   = 1.5	(mV)	        : inact tau slope
	qg   = 1.5      (mV)
	mmin=0.02	
	hmin=0.5		
	q10=3
	Rg   = 0.01 	(/ms)		: inact recov (v) 	
	Rd   = .03 	(/ms)		: inact (v)	

	:thinf  = -54	(mV)		: inact inf slope	
	thinf  = -61	(mV)		: inact inf slope	
	qinf  = 4 	(mV)		: inact inf slope 
      : qinf  =1 	(mV)		: inact inf slope 


	ena		(mV)            : must be explicitly def. in hoc
	celsius
	v 		(mV)
	h0=0.95
}


UNITS {
	(mA) = (milliamp)
	(mV) = (millivolt)
	(pS) = (picosiemens)
	(um) = (micron)
} 

ASSIGNED {
	ina 		(mA/cm2)
	thegna		(mho/cm2)
	minf 		hinf 		
	mtau (ms)	htau (ms) 
        :il      (mA/cm2)	
}
 

STATE { m h}

BREAKPOINT {
        SOLVE states METHOD cnexp
        thegna = gbar*m*m*m*h
	ina = thegna * (v - ena)
       :il   = gl * (v - el)
} 

INITIAL {
	trates(v)
	m=minf  
	h=h0
}

DERIVATIVE states {   
        trates(v)      
        m' = (minf-m)/mtau
        h' = (hinf-h)/htau
}

PROCEDURE trates(vm) {  
        LOCAL  a, b, qt
        qt=q10^((celsius-24)/10)
	a = trap0(vm,tha,Ra,qa)
	b = trap0(-vm,-tha,Rb,qa)
	mtau = 1/(a+b)/qt
        if (mtau<mmin) {mtau=mmin}
	minf = a/(a+b)

	a = trap0(vm,thi1,Rd,qd)
	b = trap0(-vm,-thi2,Rg,qg)
	htau =  1/(a+b)/qt
        if (htau<hmin) {htau=hmin}
	hinf = 1/(1+exp((vm-thinf)/qinf))
}

FUNCTION trap0(v,th,a,q) {
	if (fabs(v-th) > 1e-6) {
	        trap0 = a * (v - th) / (1 - exp(-(v - th)/q))
	} else {
	        trap0 = a * q
 	}
}	

        

