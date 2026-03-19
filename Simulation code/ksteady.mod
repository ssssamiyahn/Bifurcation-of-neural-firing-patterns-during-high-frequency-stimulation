COMMENT
set intracellular and extracellular potassium concentration steady
ENDCOMMENT

NEURON {
SUFFIX ksteady
USEION k WRITE ko, ki
RANGE ko0, ki0
}
PARAMETER {
ko0 = 3 (milli/liter)
ki0 = 106 (milli/liter)
}
STATE {
ko (milli/liter)
ki (milli/liter)
}
INITIAL {
ko = ko0
ki=ki0
}







