COMMENT
Longitudinal diffusion of potassium from bellinger et al. 2008
add glial cells uptake potassium
modeled as potassium buffering
refer to articles of H. KAGER et al. 2000, M. Bazhenov et al. 2004, Flavio Fro¡§hlich et al. 2006, Naomi Lewin et al. 2012
Zheng Lyup, 2018
ENDCOMMENT

NEURON {
	SUFFIX kdifus
	USEION k READ ik, ko WRITE ko
	:USEION k READ ik WRITE ko
	RANGE TotalBuffer, k2buf, r, width, kbath
	RANGE Dk, sp, rseg, length
}

UNITS {
  (molar) = (1/liter)
  (mM) = (millimolar)
	(um) = (micron)
	(mA) = (milliamp)
	FARADAY = (faraday) (coulomb)
	PI = (pi) (1)
}

PARAMETER {
	k2buf=0.0008
	TotalBuffer=500(mM)
	width=1(um)
	r=-1.15
	kbath=15
                	
                Dk = 1.85 (um2/ms)
	sp = 0.01 (um)
	rseg = 1.7 (um)
	extracellularVolumePerLength (um2)
	crossSectionalArea (um2)
}

ASSIGNED { 
   ik (mA/cm2)
   :ko(mM)
   Bufferspace(1)
   
   k1buf
   
   kd (/mM)  :dissociation constant for the buffer
   B0 (mM)   :the initial value of the free buffer
    }

STATE {  
      ko (mM)
      
      KBuffer(mM)
      Buffer(mM)
       
  }

INITIAL {
	extracellularVolumePerLength = PI * ( (rseg+sp)^2 - rseg^2 ) 
	crossSectionalArea = extracellularVolumePerLength
	
	
	kd=1/(1+exp((ko-kbath)/(r)))
	B0=TotalBuffer/(1+kd*ko)
	
	Buffer = B0
	KBuffer=TotalBuffer-B0
		
}


BREAKPOINT {
  SOLVE conc METHOD sparse
	
}


KINETIC conc {
	COMPARTMENT extracellularVolumePerLength {ko}
	LONGITUDINAL_DIFFUSION Dk*crossSectionalArea {ko}

  ~ ko << (ik*2*PI*rseg*(1e4)/(FARADAY))  : with or without K+ accumulation
  
  
  f_rates()
  ~ ko + Buffer <-> KBuffer (k1buf, k2buf)
}


 PROCEDURE f_rates(){
 k1buf= k2buf / (1+exp((ko-kbath)/(r)))
}



