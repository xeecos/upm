// Include doxygen-generated documentation
%include "pyupm_doxy2swig.i"
 
%module pyupm_makeblock
 
%include "../upm.i"
 
 
%feature("autodoc", "3");
 
 
%include "makeblock.h"
 
%{
 
    #include "makeblock.h"
%}
