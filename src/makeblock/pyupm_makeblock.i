// Include doxygen-generated documentation
%include "pyupm_doxy2swig.i"
 
%module pyupm_mymodule
 
%include "../upm.i"
 
 
%feature("autodoc", "3");
 
 
%include "mymodule.h"
 
%{
 
    #include "mymodule.h"
%}
